import math
import os
import csv
from datetime import datetime

import numpy as np
import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray

import torch

from .ppo_training import PPOTraining
from .actor_critic_network import action_to_shaping


class PPOTrainer(Node):
    def __init__(self):
        super().__init__("ppo_trainer")

        # ---- minimál paraméterek ----
        self.declare_parameter("train_mode", True)
        self.declare_parameter("control_hz", 10.0)
        self.declare_parameter("max_steps", 300)

        self.declare_parameter("goal_tolerance", 0.20)
        self.declare_parameter("collision_distance", 0.18)

        self.declare_parameter("lidar_bins", 12)
        self.declare_parameter("lidar_max_range", 6.0)
        self.declare_parameter("max_offset_m", 0.20)

        # warmup: első N epizódban (itt 1 epizód van, de hagyjuk egyszerűen)
        self.declare_parameter("warmup_offset_limit", 0.05)

        # topicok
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("path_topic", "/planned_path_refined")
        self.declare_parameter("params_topic", "/refiner_params")

        # run mappa
        self.declare_parameter("runs_dir", "./ppo_runs")

        # ---- beolvasás ----
        self.train_mode = bool(self.get_parameter("train_mode").value)
        self.control_hz = float(self.get_parameter("control_hz").value)
        self.max_steps = int(self.get_parameter("max_steps").value)

        self.goal_tolerance = float(self.get_parameter("goal_tolerance").value)
        self.collision_distance = float(self.get_parameter("collision_distance").value)

        self.lidar_bins = int(self.get_parameter("lidar_bins").value)
        self.lidar_max_range = float(self.get_parameter("lidar_max_range").value)
        self.max_offset_m = float(self.get_parameter("max_offset_m").value)

        self.warmup_offset_limit = float(self.get_parameter("warmup_offset_limit").value)

        self.odom_topic = str(self.get_parameter("odom_topic").value)
        self.scan_topic = str(self.get_parameter("scan_topic").value)
        self.path_topic = str(self.get_parameter("path_topic").value)
        self.params_topic = str(self.get_parameter("params_topic").value)

        self.runs_dir = str(self.get_parameter("runs_dir").value)

        # ---- run mappa létrehozás (nem ír felül!) ----
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.run_dir = os.path.join(self.runs_dir, f"run_{ts}")
        os.makedirs(self.run_dir, exist_ok=True)

        self.metrics_path = os.path.join(self.run_dir, "metrics.csv")
        self._init_metrics_file()

        # ---- bemenet cache ----
        self.last_odom = None
        self.last_scan = None
        self.last_path = None

        # ---- epizód állapot ----
        self.step_count = 0
        self.prev_dist = None

        # aktuális akció -> offset/smooth
        self.current_action = np.array([0.0, 0.0], dtype=np.float32)
        self.current_log_prob = 0.0
        self.current_offset = 0.0
        self.current_smooth = 0.0

        # ---- PPO ----
        self.state_dim = 4 + self.lidar_bins
        self.action_dim = 2
        self.trainer = PPOTraining(state_dim=self.state_dim, action_dim=self.action_dim)

        # ide mentsen
        self.trainer.save_dir = self.run_dir
        os.makedirs(self.trainer.save_dir, exist_ok=True)

        # ---- ROS IO ----
        self.sub_odom = self.create_subscription(Odometry, self.odom_topic, self.cb_odom, 20)
        self.sub_scan = self.create_subscription(LaserScan, self.scan_topic, self.cb_scan, 10)
        self.sub_path = self.create_subscription(Path, self.path_topic, self.cb_path, 10)

        self.pub_params = self.create_publisher(Float32MultiArray, self.params_topic, 10)

        # timer
        period = 1.0 / max(1e-6, self.control_hz)
        self.timer = self.create_timer(period, self.on_timer)

        mode = "TRAIN" if self.train_mode else "EVAL"
        self.get_logger().info(f"PPOTrainer indul. mode={mode}")
        self.get_logger().info(f"Run mappa: {self.run_dir}")
        self.get_logger().info("Reset nincs: 1 futás = 1 epizód, epizód végén leáll.")

    # -------------------------
    # Callbacks
    # -------------------------
    def cb_odom(self, msg: Odometry):
        self.last_odom = msg

    def cb_scan(self, msg: LaserScan):
        self.last_scan = msg

    def cb_path(self, msg: Path):
        self.last_path = msg

    # -------------------------
    # Metrics
    # -------------------------
    def _init_metrics_file(self):
        with open(self.metrics_path, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["steps", "reason", "offset", "smooth"])

    def _save_metrics(self, steps: int, reason: str):
        with open(self.metrics_path, "a", newline="") as f:
            w = csv.writer(f)
            w.writerow([steps, reason, f"{self.current_offset:.4f}", f"{self.current_smooth:.4f}"])

    # -------------------------
    # Main loop
    # -------------------------
    def on_timer(self):
        # kell adat
        if self.last_odom is None or self.last_scan is None or self.last_path is None:
            return
        if len(self.last_path.poses) < 2:
            return

        # epizód eleje: választunk paramot
        if self.step_count == 0:
            self.pick_params_for_episode()

        # állapot
        state, info = self.build_state(self.last_odom, self.last_scan, self.last_path)
        dist = info["dist_goal"]
        min_r = info["min_range"]

        # done feltételek + reward
        if dist < self.goal_tolerance:
            reward, done, reason = 50.0, True, "goal"
        elif min_r < self.collision_distance:
            reward, done, reason = -50.0, True, "collision"
        elif self.step_count >= self.max_steps:
            reward, done, reason = -10.0, True, "timeout"
        else:
            progress = self.prev_dist - dist
            self.prev_dist = dist
            reward, done, reason = (2.0 * progress - 0.01), False, "running"

        # TRAIN: store
        if self.train_mode:
            self.trainer.store(state, self.current_action, self.current_log_prob, reward, done)

        self.step_count += 1

        # paramok publish minden tickben (hogy biztos kapja a refiner)
        msg = Float32MultiArray()
        msg.data = [self.current_offset, self.current_smooth]
        self.pub_params.publish(msg)

        # epizód vége
        if done:
            if self.train_mode:
                self.trainer.finish_episode()

            self.get_logger().info(f"[EP END] steps={self.step_count} reason={reason}")
            self._save_metrics(self.step_count, reason)

            # itt vége (reset nincs)
            rclpy.shutdown()

    def pick_params_for_episode(self):
        """Egyszer kiválasztjuk az offset/smooth paramot."""
        state0, info0 = self.build_state(self.last_odom, self.last_scan, self.last_path)
        st = torch.tensor(state0, dtype=torch.float32)

        with torch.no_grad():
            dist, _ = self.trainer.policy(st)
            action = dist.sample() if self.train_mode else dist.mean
            self.current_log_prob = float(dist.log_prob(action).sum(-1).item())

        action_np = action.squeeze(0).cpu().numpy().astype(np.float32)
        self.current_action = action_np

        off, sm = action_to_shaping(torch.tensor(action_np), max_offset_m=self.max_offset_m)

        # óvatos limit (BSc-s egyszerű védelem)
        off = float(max(-self.warmup_offset_limit, min(self.warmup_offset_limit, float(off))))

        self.current_offset = float(off)
        self.current_smooth = float(sm)

        # induló progress referencia
        self.prev_dist = float(info0["dist_goal"])

        self.get_logger().info(f"[EP START] off={self.current_offset:.3f} sm={self.current_smooth:.2f}")

        # publish egyszer az elején is
        msg = Float32MultiArray()
        msg.data = [self.current_offset, self.current_smooth]
        self.pub_params.publish(msg)

    def build_state(self, odom: Odometry, scan: LaserScan, path: Path):
        rx = odom.pose.pose.position.x
        ry = odom.pose.pose.position.y

        v = float(odom.twist.twist.linear.x)
        w = float(odom.twist.twist.angular.z)

        gx = path.poses[-1].pose.position.x
        gy = path.poses[-1].pose.position.y
        dist_goal = math.hypot(gx - rx, gy - ry)

        ranges = np.array(scan.ranges, dtype=np.float32)
        ranges = np.where(np.isfinite(ranges), ranges, self.lidar_max_range)
        ranges = np.clip(ranges, 0.0, self.lidar_max_range)

        if len(ranges) == 0:
            lidar_vec = [1.0] * self.lidar_bins
            min_range = self.lidar_max_range
        else:
            min_range = float(np.min(ranges))
            n = len(ranges)
            step = max(1, n // self.lidar_bins)
            lidar_vec = []
            for b in range(self.lidar_bins):
                a = b * step
                c = min(n, (b + 1) * step)
                m = float(np.min(ranges[a:c])) if a < n else self.lidar_max_range
                lidar_vec.append(m / self.lidar_max_range)

        state = np.array([dist_goal, v, w, float(min_range)] + lidar_vec, dtype=np.float32)
        info = {"dist_goal": float(dist_goal), "min_range": float(min_range)}
        return state, info


def main(args=None):
    rclpy.init(args=args)
    node = PPOTrainer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
