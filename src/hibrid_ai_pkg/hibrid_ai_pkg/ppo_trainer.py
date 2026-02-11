import math
import os
import csv
import shutil
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

        # minimál paraméterek 
        self.declare_parameter("train_mode", True)
        self.declare_parameter("control_hz", 10.0)

        # epizód
        self.declare_parameter("max_steps", 1200)
        self.declare_parameter("goal_tolerance", 0.50)
        self.declare_parameter("collision_distance", 0.18)
        self.declare_parameter("min_steps_for_goal", 50)

        # state
        self.declare_parameter("lidar_bins", 12)
        self.declare_parameter("lidar_max_range", 6.0)

        # shaping határ
        self.declare_parameter("max_offset_m", 0.05)
        self.declare_parameter("offset_limit", 0.02)   # egyszerű limit
        self.declare_parameter("smooth_max", 0.25)     # fix plafon

        # topicok
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("path_topic", "/planned_path_refined")
        self.declare_parameter("params_topic", "/refiner_params")

        # mentés
        self.declare_parameter("runs_dir", "./ppo_runs")

        # beolvasás 
        self.train_mode = bool(self.get_parameter("train_mode").value)
        self.control_hz = float(self.get_parameter("control_hz").value)

        self.max_steps = int(self.get_parameter("max_steps").value)
        self.goal_tolerance = float(self.get_parameter("goal_tolerance").value)
        self.collision_distance = float(self.get_parameter("collision_distance").value)
        self.min_steps_for_goal = int(self.get_parameter("min_steps_for_goal").value)

        self.lidar_bins = int(self.get_parameter("lidar_bins").value)
        self.lidar_max_range = float(self.get_parameter("lidar_max_range").value)

        self.max_offset_m = float(self.get_parameter("max_offset_m").value)
        self.offset_limit = float(self.get_parameter("offset_limit").value)
        self.smooth_max = float(self.get_parameter("smooth_max").value)

        self.odom_topic = str(self.get_parameter("odom_topic").value)
        self.scan_topic = str(self.get_parameter("scan_topic").value)
        self.path_topic = str(self.get_parameter("path_topic").value)
        self.params_topic = str(self.get_parameter("params_topic").value)

        self.runs_dir = str(self.get_parameter("runs_dir").value)
        os.makedirs(self.runs_dir, exist_ok=True)

        #  run mappa (nem ír felül) 
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.run_dir = os.path.join(self.runs_dir, f"run_{ts}")
        os.makedirs(self.run_dir, exist_ok=True)

        # fájlok
        self.run_metrics_path = os.path.join(self.run_dir, "metrics.csv")
        self.global_metrics_path = os.path.join(self.runs_dir, "global_metrics.csv")
        self.best_meta_path = os.path.join(self.runs_dir, "best_meta.csv")
        self.best_model_path = os.path.join(self.runs_dir, "best_latest.pth")
        self.latest_global_path = os.path.join(self.runs_dir, "latest_global.pth")

        self._init_csv_if_needed(self.run_metrics_path, header=[
            "steps", "reason", "offset", "smooth", "dist_goal", "min_range", "progress", "score"
        ])
        self._init_csv_if_needed(self.global_metrics_path, header=[
            "run", "steps", "reason", "offset", "smooth", "dist_goal", "min_range", "progress", "score", "model"
        ])
        self._init_csv_if_needed(self.best_meta_path, header=[
            "run", "steps", "reason", "progress", "score", "src_model"
        ])

        # bemenet cach
        self.last_odom = None
        self.last_scan = None
        self.last_path = None

        # epizód állapot
        self.step_count = 0
        self.prev_dist = None
        self.progress_sum = 0.0

        # aktuális paramok
        self.current_action = np.array([0.0, 0.0], dtype=np.float32)
        self.current_log_prob = 0.0
        self.current_offset = 0.0
        self.current_smooth = 0.0

        # PPO
        self.state_dim = 4 + self.lidar_bins
        self.action_dim = 2
        self.trainer = PPOTraining(state_dim=self.state_dim, action_dim=self.action_dim)

        # mentés: ide
        self.trainer.save_dir = self.run_dir
        os.makedirs(self.trainer.save_dir, exist_ok=True)

        # legyen 1 epizód = 1 mentés
        self.trainer.save_freq = 1

        # induláskor mindig BEST betöltés, ha van 
        if self.train_mode:
            self._load_best_if_exists()

        self.sub_odom = self.create_subscription(Odometry, self.odom_topic, self.cb_odom, 20)
        self.sub_scan = self.create_subscription(LaserScan, self.scan_topic, self.cb_scan, 10)
        self.sub_path = self.create_subscription(Path, self.path_topic, self.cb_path, 10)

        self.pub_params = self.create_publisher(Float32MultiArray, self.params_topic, 10)

        period = 1.0 / max(1e-6, self.control_hz)
        self.timer = self.create_timer(period, self.on_timer)

        mode = "TRAIN" if self.train_mode else "EVAL"
        self.get_logger().info(f"PPOTrainer indul. mode={mode}")
        self.get_logger().info(f"Run mappa: {self.run_dir}")
        self.get_logger().info("1 launch = 1 epizód, epizód végén shutdown.")

    # Callbacks
    def cb_odom(self, msg: Odometry):
        self.last_odom = msg

    def cb_scan(self, msg: LaserScan):
        self.last_scan = msg

    def cb_path(self, msg: Path):
        self.last_path = msg

    # CSV helper
    def _init_csv_if_needed(self, path: str, header: list):
        if os.path.exists(path):
            return
        with open(path, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(header)

    def _append_csv(self, path: str, row: list):
        with open(path, "a", newline="") as f:
            w = csv.writer(f)
            w.writerow(row)

    # Best load
    def _load_best_if_exists(self):
        if not os.path.exists(self.best_model_path):
            self.get_logger().info("[LOAD] best_latest.pth nincs, indul nulláról.")
            return
        try:
            # PPOTraining/ActorCriticNetwork ezt használja betöltéshez
            self.trainer.policy.save_file = self.best_model_path
            self.trainer.policy.load_from_file()

            # old_policy legyen azonos (PPO stabil)
            self.trainer.copy_policy()

            self.get_logger().info(f"[LOAD] Betöltve: {self.best_model_path}")
        except Exception as e:
            self.get_logger().error(f"[LOAD] Betöltés hiba: {e}")

    # Main loop
    def on_timer(self):
        if self.last_odom is None or self.last_scan is None or self.last_path is None:
            return
        if len(self.last_path.poses) < 2:
            return

        # epizód eleje: egyszer választunk paramot
        if self.step_count == 0:
            self.pick_params_for_episode()

        # state
        state, info = self.build_state(self.last_odom, self.last_scan, self.last_path)
        dist = info["dist_goal"]
        min_r = info["min_range"]

        # progress számolás (összeg)
        if self.prev_dist is not None:
            self.progress_sum += (self.prev_dist - dist)
        self.prev_dist = dist

        can_be_goal = (self.step_count >= self.min_steps_for_goal)

        # done
        if can_be_goal and dist < self.goal_tolerance:
            reward, done, reason = 50.0, True, "goal"
        elif min_r < self.collision_distance:
            reward, done, reason = -50.0, True, "collision"
        elif self.step_count >= self.max_steps:
            reward, done, reason = -10.0, True, "timeout"
        else:
            # egyszerű reward
            reward, done, reason = (2.0 * (self.prev_dist - dist) - 0.01), False, "running"

        # store
        if self.train_mode:
            self.trainer.store(state, self.current_action, self.current_log_prob, reward, done)

        self.step_count += 1

        # publish param minden tickben
        msg = Float32MultiArray()
        msg.data = [self.current_offset, self.current_smooth]
        self.pub_params.publish(msg)

        # epizód vége
        if done:
            self.finish_and_exit(reason, dist, min_r)

    # Episode begin/end
    def pick_params_for_episode(self):
        state0, info0 = self.build_state(self.last_odom, self.last_scan, self.last_path)
        st = torch.tensor(state0, dtype=torch.float32)

        with torch.no_grad():
            dist, _ = self.trainer.policy(st)
            action = dist.sample() if self.train_mode else dist.mean
            self.current_log_prob = float(dist.log_prob(action).sum(-1).item())

        action_np = action.squeeze(0).cpu().numpy().astype(np.float32)
        self.current_action = action_np

        off, sm = action_to_shaping(torch.tensor(action_np), max_offset_m=self.max_offset_m)

        # egyszerű limit
        off = float(max(-self.offset_limit, min(self.offset_limit, float(off))))
        sm = float(max(0.0, min(self.smooth_max, float(sm))))

        self.current_offset = off
        self.current_smooth = sm

        self.prev_dist = float(info0["dist_goal"])
        self.progress_sum = 0.0

        self.get_logger().info(f"[EP START] off={self.current_offset:.3f} sm={self.current_smooth:.2f}")

        # publish egyszer az elején is
        msg = Float32MultiArray()
        msg.data = [self.current_offset, self.current_smooth]
        self.pub_params.publish(msg)

    def finish_and_exit(self, reason: str, dist_goal: float, min_range: float):
        # tanítás + mentés (PPOTraining intézi)
        model_path = ""
        if self.train_mode:
            self.trainer.finish_episode()

            # PPOTraining most mentett egy pth-t a save_dir-be (run_dir)
            # egyszerűen megkeressük a legfrissebb .pth-t (ami nem latest)
            model_path = self._find_latest_model_in_run()

            # legyen "legutóbbi futás" modell
            if model_path and os.path.exists(model_path):
                shutil.copyfile(model_path, self.latest_global_path)

        # score
        score = self._score(reason, self.progress_sum, self.step_count)

        # run metrics (1 sor)
        self._append_csv(self.run_metrics_path, [
            self.step_count,
            reason,
            f"{self.current_offset:.4f}",
            f"{self.current_smooth:.4f}",
            f"{dist_goal:.4f}",
            f"{min_range:.4f}",
            f"{self.progress_sum:.4f}",
            f"{score:.4f}",
        ])

        # global metrics (1 sor)
        self._append_csv(self.global_metrics_path, [
            os.path.basename(self.run_dir),
            self.step_count,
            reason,
            f"{self.current_offset:.4f}",
            f"{self.current_smooth:.4f}",
            f"{dist_goal:.4f}",
            f"{min_range:.4f}",
            f"{self.progress_sum:.4f}",
            f"{score:.4f}",
            model_path,
        ])

        # best frissítés (ha van mentett modell)
        if model_path and os.path.exists(model_path):
            self._maybe_update_best(reason, self.step_count, self.progress_sum, score, model_path)

        self.get_logger().info(
            f"[EP END] steps={self.step_count} reason={reason} progress={self.progress_sum:.3f} score={score:.2f}"
        )
        self.get_logger().info("Leáll (1 launch = 1 epizód).")
        rclpy.shutdown()

    def _find_latest_model_in_run(self) -> str:
        try:
            files = [f for f in os.listdir(self.run_dir) if f.endswith(".pth")]
            files = [f for f in files if f != "latest.pth"]
            if not files:
                return ""
            files.sort(key=lambda x: os.path.getmtime(os.path.join(self.run_dir, x)))
            return os.path.join(self.run_dir, files[-1])
        except Exception:
            return ""

    def _score(self, reason: str, progress: float, steps: int) -> float:
        # cél: goal legyen előny, collision nagy bünti
        if reason == "goal":
            return 1000.0 + progress - 0.1 * steps
        if reason == "collision":
            return -1000.0 + progress - 0.1 * steps
        # timeout / egyéb
        return progress - 0.1 * steps

    def _read_best_score(self) -> float:
        # best_meta.csv utolsó sorának score-ja
        try:
            if not os.path.exists(self.best_meta_path):
                return -1e18
            with open(self.best_meta_path, "r", newline="") as f:
                rows = list(csv.reader(f))
            if len(rows) < 2:
                return -1e18
            last = rows[-1]
            # header: run,steps,reason,progress,score,src_model
            return float(last[4])
        except Exception:
            return -1e18

    def _maybe_update_best(self, reason: str, steps: int, progress: float, score: float, model_path: str):
        best_score = self._read_best_score()

        if score <= best_score:
            return

        # best_latest.pth = ez a modell
        try:
            shutil.copyfile(model_path, self.best_model_path)
            self._append_csv(self.best_meta_path, [
                os.path.basename(self.run_dir),
                steps,
                reason,
                f"{progress:.4f}",
                f"{score:.4f}",
                model_path
            ])
            self.get_logger().info(f"[BEST] Frissült! score={score:.2f} -> {self.best_model_path}")
        except Exception as e:
            self.get_logger().error(f"[BEST] mentés hiba: {e}")

    # State
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
