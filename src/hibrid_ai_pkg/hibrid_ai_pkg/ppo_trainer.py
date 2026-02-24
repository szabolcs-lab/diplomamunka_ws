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

from geometry_msgs.msg import PointStamped
from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_point

from rclpy.duration import Duration

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
        self.declare_parameter("path_topic", "/planned_path_smoother")
        self.declare_parameter("params_topic", "/smoother_params")

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

        # run mappa (nem ír felül)
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.run_dir = os.path.join(self.runs_dir, f"run_{ts}")
        os.makedirs(self.run_dir, exist_ok=True)

        # fájlok
        self.run_metrics_path = os.path.join(self.run_dir, "metrics.csv")
        self.global_metrics_path = os.path.join(self.runs_dir, "global_metrics.csv")
        self.best_metric_path = os.path.join(self.runs_dir, "best_metrics.csv")
        self.best_model_path = os.path.join(self.runs_dir, "best_latest.pth")
        self.latest_global_path = os.path.join(self.runs_dir, "latest_global.pth")

        self._init_csv_if_needed(self.run_metrics_path, header=["lepesek_szama", "befejezes_oka", "eltolas_meterben", "simitas", "celtol_valo_tavolsag (m)", 
                                                                "legkozelebbi_akadaly_tavolsag", "ossz_haladas", "kapott_pontszam"])
        
        self._init_csv_if_needed(self.global_metrics_path, header=["futas_azonosito", "lepesek_szama", "befejezes_oka", "eltolas_meterben", "simitas", 
                                                                   "celtol_valo_tavolsag (m)", "legkozelebbi_akadaly_tavolsag", 
                                                                   "ossz_haladas", "kapott_pontszam", "modell_fajl"])
        
        self._init_csv_if_needed(self.best_metric_path, header=["futas_azonosito", "lepesek_szama", "befejezes_oka", "legkozelebbi_akadaly_tavolsag", "ossz_haladas", "kapott_pontszam", "forras_modell"])

        # bemenet cach
        self.last_odom = None
        self.last_scan = None
        self.last_path = None

        # epizód állapot
        self.step_count = 0
        self.prev_distance_goal = None
        self.progress_sum = 0.0

        # aktuális paramok
        self.current_action = np.array([0.0, 0.0], dtype=np.float32)
        self.current_log_prob = 0.0
        self.current_offset = 0.0
        self.current_smooth = 0.0

        # PPO
        #self.state_dim = 4 + self.lidar_bins
        self.state_dim = 5 + self.lidar_bins
        self.action_dim = 2
        self.trainer = PPOTraining(state_dim=self.state_dim, action_dim=self.action_dim)

        # mentés: ide
        self.trainer.save_dir = self.run_dir
        os.makedirs(self.trainer.save_dir, exist_ok=True)

        # legyen 1 epizód = 1 mentés
        self.trainer.save_gyakorisag = 1

        # induláskor mindig BEST betöltés, ha van
        if self.train_mode:
            self._load_best_if_exists()

        self.sub_odom = self.create_subscription(Odometry, self.odom_topic, self.cb_odom, 20)
        self.sub_scan = self.create_subscription(LaserScan, self.scan_topic, self.cb_scan, 10)
        self.sub_path = self.create_subscription(Path, self.path_topic, self.cb_path, 10)

        self.pub_params = self.create_publisher(Float32MultiArray, self.params_topic, 10)

        period = 1.0 / max(1e-6, self.control_hz)
        self.timer = self.create_timer(period, self.on_timer)
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

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

        # epizód eleje: egyszer választok paramot
        if self.step_count == 0:
            self.pick_params_for_episode()

        # state
        state, info = self.build_state(self.last_odom, self.last_scan, self.last_path)
        
        if state is None or info is None:
            return
        
        distance_goal = info["distance_goal"]
        min_r = info["min_range"]
        cross_track_error = info["cross_track_error"]

        # progress / delta (előbb számolom, utána frissítem prev-et!)
        delta_distance_goal = 0.0
        if self.prev_distance_goal is not None:
            delta_distance_goal = (self.prev_distance_goal - distance_goal)
            self.progress_sum += delta_distance_goal
            
        self.prev_distance_goal = distance_goal

        can_be_goal = (self.step_count >= self.min_steps_for_goal)

        # done
        if can_be_goal and distance_goal < self.goal_tolerance:
            reward, done, reason = 50.0, True, "goal"
            
        elif min_r < self.collision_distance:
            reward, done, reason = -50.0, True, "collision"
            
        elif self.step_count >= self.max_steps:
            # ha már közel a célhoz, adjunk még időt
            if distance_goal < 2.0:
                reward, done, reason = -0.01, False, "running"   # még fut
            else:
                reward, done, reason = -10.0, True, "timeout"
            
        else:
            # egyszerű reward
            # (2.0 * delta_distance_goal - 0.01 - 0.2 * cross_track_error - 0.02 * abs(self.current_offset))
            reward, done, reason = (2.0 * delta_distance_goal - 0.01 - 0.15 * cross_track_error - 0.005 * abs(self.current_offset)), False, "running"

        # store
        if self.train_mode:
            self.trainer.store(state, self.current_action, self.current_log_prob, reward, done)

        self.step_count += 1

        # publish param
        msg = Float32MultiArray()
        msg.data = [self.current_offset, self.current_smooth]
        self.pub_params.publish(msg)

        # epizód vége
        if done:
            self.finish_and_exit(reason, distance_goal, min_r)
            
        if self.step_count % 20 == 0:
            robot_x, robot_y = self.robot_xy_in_map(self.last_odom)
            self.get_logger().info(
                f"dist_goal={distance_goal:.3f} tol={self.goal_tolerance:.3f} "
                f"robot_map=({robot_x:.2f},{robot_y:.2f}) "
                f"path_goal=({self.last_path.poses[-1].pose.position.x:.2f},{self.last_path.poses[-1].pose.position.y:.2f})")

    # Episode begin/end
    def pick_params_for_episode(self):
        state0, info0 = self.build_state(self.last_odom, self.last_scan, self.last_path)
        st = torch.tensor(state0, dtype=torch.float32)

        with torch.no_grad():
            action_distribution, _ = self.trainer.policy(st)
            action = action_distribution.sample() if self.train_mode else action_distribution.mean
            self.current_log_prob = float(action_distribution.log_prob(action).sum(-1).item())

        action_np = action.squeeze(0).cpu().numpy().astype(np.float32)
        self.current_action = action_np

        off, sm = action_to_shaping(torch.tensor(action_np), max_offset_meter=self.max_offset_m)

        # egyszerű limit
        off = float(max(-self.offset_limit, min(self.offset_limit, float(off))))
        sm = float(max(0.0, min(self.smooth_max, float(sm))))

        self.current_offset = off
        self.current_smooth = sm

        self.prev_distance_goal = float(info0["distance_goal"])
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
            #  megkeresem a legfrissebb .pth-t (ami nem latest)
            model_path = self._find_latest_model_in_run()

            # legyen "legutóbbi futás" modell
            if model_path and os.path.exists(model_path):
                shutil.copyfile(model_path, self.latest_global_path)

        # score
        score = self._score(reason, self.progress_sum, self.step_count)

        # run metrics (1 sor)
        self._append_csv(self.run_metrics_path, [self.step_count, reason, f"{self.current_offset:.4f}", f"{self.current_smooth:.4f}", f"{dist_goal:.4f}", 
                                                 f"{min_range:.4f}", f"{self.progress_sum:.4f}", f"{score:.4f}"])

        # global metrics (1 sor)
        self._append_csv(self.global_metrics_path, [os.path.basename(self.run_dir), self.step_count, reason, f"{self.current_offset:.4f}", f"{self.current_smooth:.4f}", 
                                                    f"{dist_goal:.4f}", f"{min_range:.4f}", f"{self.progress_sum:.4f}", f"{score:.4f}", model_path])

        # best frissítés (ha van mentett modell)
        if model_path and os.path.exists(model_path):
            self._maybe_update_best(reason, self.step_count, min_range, self.progress_sum, score, model_path)


        self.get_logger().info(f"[EP END] steps={self.step_count} reason={reason} progress={self.progress_sum:.3f} score={score:.2f}")
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

    def _score(self, reason: str, progress: float, steps: int):
        # cél: goal legyen előny, collision nagy bünti
        
        if reason == "goal":
            return 1000.0 + progress - 0.1 * steps
        
        if reason == "collision":
            return -1000.0 + progress - 0.1 * steps
        
        # timeout / egyéb
        return progress - 0.1 * steps

    def _read_best_row(self):
        """
        Visszaadja a best_metrics.csv utolsó (legjobbként eltárolt) sorát.
        Ha még nincs best, None.
        """
        if not os.path.exists(self.best_metric_path):
            return None

        with open(self.best_metric_path, "r", newline="") as f:
            rows = list(csv.reader(f))

        if len(rows) < 2:   # csak header van
            return None

        return rows[-1]
        
    def _maybe_update_best(self,reason: str, steps: int, min_range: float, progress: float, score: float,model_path: str):
        #csak GOAL-ból választunk best-et
        if reason != "goal":
            return

        best_row = self._read_best_row()

        # Ha még nincs best: ez az első goal - automatikusan best
        if best_row is None:
            self.save_as_best(steps, min_range, progress, score, model_path)
            return

        # best_row mezők a header alapján:
        # ["futas_azonosito","lepesek_szama","befejezes_oka","legkozelebbi_akadaly_tavolsag","ossz_haladas","kapott_pontszam","forras_modell"]
        best_steps = int(best_row[1])
        best_min_range = float(best_row[3])

        # kevesebb lépés = jobb
        if steps < best_steps:
            self.save_as_best(steps, min_range, progress, score, model_path)
            return

        # ha több lépés, nem jobb
        if steps > best_steps:
            return

        # döntetlen: nagyobb min_range = jobb
        if min_range > best_min_range:
            self.save_as_best(steps, min_range, progress, score, model_path)
            return

        # ha min_range se jobb, akkor nem frissítünk
        return



    # State
    def build_state(self, odom: Odometry, scan: LaserScan, path: Path):
        # robot pozíció map-ben (mert a path is map-ben van!)
        robot_xy = self.robot_xy_in_map(odom)
        if robot_xy[0] is None:
            
            # nincs TF -> ne számolj hülyeséget
            return None, None

        robot_x, robot_y = robot_xy

        robot_speed = float(odom.twist.twist.linear.x)
        robot_turn_speed = float(odom.twist.twist.angular.z)

        goal_x = float(path.poses[-1].pose.position.x)
        goal_y = float(path.poses[-1].pose.position.y)
        distance_goal = math.hypot(goal_x - robot_x, goal_y - robot_y)

        ranges = np.array(scan.ranges, dtype=np.float32)
        ranges = np.where(np.isfinite(ranges), ranges, self.lidar_max_range)
        ranges = np.clip(ranges, 0.0, self.lidar_max_range)

        if len(ranges) == 0:
            lidar_vector = [1.0] * self.lidar_bins
            min_range = float(self.lidar_max_range)
        else:
            min_range = float(np.min(ranges))
            total_lidar_points = len(ranges)
            points_per_bin = max(1, total_lidar_points // self.lidar_bins)

            lidar_vector = []
            for i in range(self.lidar_bins):
                start_index = i * points_per_bin
                end_index = min(total_lidar_points, (i + 1) * points_per_bin)

                if start_index < total_lidar_points:
                    min_distance_i = float(np.min(ranges[start_index:end_index]))
                else:
                    min_distance_i = self.lidar_max_range

                lidar_vector.append(min_distance_i / self.lidar_max_range)
                
        cross_track_error = self.calc_cross_track_error_map_xy(robot_x, robot_y, path)
        
        norm_goal_distance = min(distance_goal / 20.0, 1.0)          # 20m felett 1.0
        norm_cross_track_error   = min(cross_track_error / 2.0, 1.0)       # 2m felett 1.0

        norm_linear_speed  = np.clip(robot_speed / 1.0, -1.0, 1.0)      # ha ~1 m/s a max
        norm_angular_speed  = np.clip(robot_turn_speed / 1.5, -1.0, 1.0) # ha ~1.5 rad/s a max

        norm_min_lidar_range  = np.clip(min_range / self.lidar_max_range, 0.0, 1.0)

        state = np.array([norm_goal_distance, norm_linear_speed , norm_angular_speed , norm_min_lidar_range ,
                          norm_cross_track_error ] + lidar_vector, dtype=np.float32)
                
        info = {"distance_goal": float(distance_goal), "min_range": float(min_range), "cross_track_error": float(cross_track_error)}
        
        return state, info
    
    def calc_cross_track_error_map_xy(self, robot_x: float, robot_y: float, path: Path):
        """Távolság a robot (map) és a Path legközelebbi pontja között (méterben)."""

        if path is None or len(path.poses) == 0:
            return 0.0

        min_distance_meter  = 1e9
        for pose_stamped in path.poses:
            path_point_x = float(pose_stamped.pose.position.x)  # map
            path_point_y = float(pose_stamped.pose.position.y)  # map
            distance_meter = math.hypot(path_point_x - robot_x, path_point_y - robot_y)
            
            if distance_meter < min_distance_meter :
                min_distance_meter  = distance_meter

        return float(min_distance_meter )
    
    def save_as_best(self, steps: int, min_range: float, progress: float, score: float, model_path: str):
        try:
            shutil.copyfile(model_path, self.best_model_path)

            self._append_csv(self.best_metric_path, [os.path.basename(self.run_dir), steps,"goal", f"{min_range:.4f}", f"{progress:.4f}", f"{score:.4f}", model_path])

            self.get_logger().info(f"A best frissült! steps={steps} min_range={min_range:.3f} -> {self.best_model_path}")
            
        except Exception as e:
            self.get_logger().error(f"A best mentése során hiba történt: {e}")
            
            
    def robot_xy_in_map(self, odom: Odometry):
        """
        Odomból robot pozícióját átszámolja map frame-be TF2-vel.
        Visszaad (x_map, y_map) vagy (None, None) tupleült ha nincs TF.
        """
        odom_point = PointStamped()
        odom_point.header.frame_id = odom.header.frame_id  # "odom"
        now = self.get_clock().now()
        odom_point.header.stamp = now.to_msg()

        odom_point.point.x = float(odom.pose.pose.position.x)
        odom_point.point.y = float(odom.pose.pose.position.y)
        odom_point.point.z = 0.0

        try:
            tf = self.tf_buffer.lookup_transform("map",odom_point .header.frame_id, now,timeout=Duration(seconds=0.2))
            map_point= do_transform_point(odom_point , tf)
            
            return float(map_point.point.x), float(map_point.point.y)

        except Exception as e:
            self.get_logger().warn(f"TF hiba: {e}")
            return None, None



def main(args=None):
    rclpy.init(args=args)
    node = PPOTrainer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
