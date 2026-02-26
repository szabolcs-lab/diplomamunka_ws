import csv
import math
import os
import shutil
from datetime import datetime
import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import PointStamped, Twist
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import LaserScan
from rcl_interfaces.msg import Parameter as RosParameter
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.msg import ParameterValue
from rcl_interfaces.srv import SetParameters
from tf2_geometry_msgs import do_transform_point
from tf2_ros import Buffer, TransformListener
import torch
from .ppo_training import PPOTraining


class PPOTrainer(Node):
    """PPO tréner node, ami epizódonként MPPI paramokat állít és metrikákat ment."""

    def __init__(self):
        super().__init__("ppo_trainer")

        self.declare_parameter("train_mode", True)
        self.is_training = bool(self.get_parameter("train_mode").value)
        
        self.declare_parameter("control_hz", 10.0)
        self.control_hz = float(self.get_parameter("control_hz").value)
        
        self.declare_parameter("max_steps", 1200)
        self.max_steps = int(self.get_parameter("max_steps").value)
        
        self.declare_parameter("goal_tolerance", 0.50)
        self.goal_tolerance_m = float(self.get_parameter("goal_tolerance").value)
        
        self.declare_parameter("collision_distance", 0.18)
        self.collision_distance_m = float(self.get_parameter("collision_distance").value)
        
        self.declare_parameter("min_steps_for_goal", 50)
        self.min_steps_for_goal = int(self.get_parameter("min_steps_for_goal").value)
        
        self.declare_parameter("lidar_bins", 12)
        self.lidar_bins = int(self.get_parameter("lidar_bins").value)
        
        self.declare_parameter("lidar_max_range", 6.0)
        self.lidar_max_range_m = float(self.get_parameter("lidar_max_range").value)
        
        #######################
        self.declare_parameter("near_obstacle_distance", 0.7)   #0.6 m
        self.near_obstacle_distance_m = float(self.get_parameter("near_obstacle_distance").value)
        
        self.declare_parameter("near_obstacle_weight", 3.0) #2.0
        self.near_obstacle_weight = float(self.get_parameter("near_obstacle_weight").value)
        ######################

        self.declare_parameter("odom_topic", "/odom")
        self.odom_topic = str(self.get_parameter("odom_topic").value)
        
        self.declare_parameter("scan_topic", "/scan")
        self.scan_topic = str(self.get_parameter("scan_topic").value)
        
        self.declare_parameter("path_topic", "/planned_path_dilated")
        self.path_topic = str(self.get_parameter("path_topic").value)
        
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.cmd_vel_topic = str(self.get_parameter("cmd_vel_topic").value)
     
        self.declare_parameter("controller_server_node", "/controller_server")
        self.controller_server_node = str(self.get_parameter("controller_server_node").value)
        
        self.declare_parameter("stuck_window_steps", 120)
        self.stuck_window_steps = int(self.get_parameter("stuck_window_steps").value)
        
        self.declare_parameter("stuck_delta_eps", 0.002)
        self.stuck_delta_eps = float(self.get_parameter("stuck_delta_eps").value)
        
        self.declare_parameter("energy_weight", 0.005)
        self.energy_weight = float(self.get_parameter("energy_weight").value)
         
        self.declare_parameter("runs_dir", "./ppo_runs")
        self.runs_dir = str(self.get_parameter("runs_dir").value)
        os.makedirs(self.runs_dir, exist_ok=True)   

        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.run_dir = os.path.join(self.runs_dir, f"run_{timestamp}")
        os.makedirs(self.run_dir, exist_ok=True)

        self.run_metrics_csv = os.path.join(self.run_dir, "metrics.csv")
        self.global_metrics_csv = os.path.join(self.runs_dir, "global_metrics.csv")
        self.best_metrics_csv = os.path.join(self.runs_dir, "best_metrics.csv")
        self.best_model_path = os.path.join(self.runs_dir, "best_latest.pth")
        self.latest_global_model_path = os.path.join(self.runs_dir, "latest_global.pth")

        self._init_csv_if_missing(self.run_metrics_csv,header=["lepesek_szama","befejezes_oka", "mppi_vx_max","mppi_cost_weight",
                                                               "celtol_valo_tavolsag (m)","legkozelebbi_akadaly_tavolsag", "ossz_haladas",
                                                               "sebessegvaltozas_energia","kapott_pontszam"])

        self._init_csv_if_missing(self.global_metrics_csv,header=["futas_azonosito","lepesek_szama", "befejezes_oka","mppi_vx_max", "mppi_cost_weight",
                                                                  "celtol_valo_tavolsag (m)","legkozelebbi_akadaly_tavolsag","ossz_haladas",
                                                                  "sebessegvaltozas_energia","kapott_pontszam","modell_fajl"])

        self._init_csv_if_missing(self.best_metrics_csv,header=["futas_azonosito","lepesek_szama","befejezes_oka","legkozelebbi_akadaly_tavolsag",
                                                                "ossz_haladas","sebessegvaltozas_energia","kapott_pontszam","forras_modell"])

        # Bejövő üzenetek cache
        self.latest_odom = None
        self.latest_scan = None
        self.latest_path = None
        self.latest_cmd_vel = None

        # Epizód állapot
        self.step_index = 0
        self.previous_goal_distance_m = None
        self.total_progress_m = 0.0

        self.total_energy = 0.0
        self.previous_cmd_v = None
        self.previous_cmd_w = None

        self.stuck_steps_count = 0

        # PPO action + logprob
        self.current_action = np.array([0.0, 0.0], dtype=np.float32)
        self.current_action_logprob = 0.0

        # MPPI paramok
        self.episode_vx_max = 0.0
        self.episode_cost_weight = 0.0

        # PPO tréner
        self.state_dim = 5 + self.lidar_bins
        self.action_dim = 2
        self.ppo_trainer = PPOTraining(state_dim=self.state_dim, action_dim=self.action_dim)
        self.ppo_trainer.save_dir = self.run_dir
        os.makedirs(self.ppo_trainer.save_dir, exist_ok=True)
        self.ppo_trainer.save_gyakorisag = 1  # 1 epizód = 1 mentés

        if self.is_training:
            self._load_best_model_if_exists()

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.mppi_set_params_client = self.create_client(SetParameters, f"{self.controller_server_node}/set_parameters")
  
        qos_path = QoSProfile(depth=10)
        qos_path.reliability = ReliabilityPolicy.RELIABLE
        qos_path.durability = DurabilityPolicy.TRANSIENT_LOCAL
        
        qos_cmd = QoSProfile(depth=20)
        qos_cmd.reliability = ReliabilityPolicy.RELIABLE
        qos_cmd.durability = DurabilityPolicy.VOLATILE
        
        qos_scan = QoSProfile(depth=10)
        qos_scan.reliability = ReliabilityPolicy.RELIABLE
        qos_scan.durability = DurabilityPolicy.VOLATILE
        
        qos_odom = QoSProfile(depth=20)
        qos_odom.reliability = ReliabilityPolicy.RELIABLE
        qos_odom.durability = DurabilityPolicy.VOLATILE

        self.sub_odom = self.create_subscription(Odometry, self.odom_topic, self.odom_callback, qos_odom)
        self.sub_scan = self.create_subscription(LaserScan, self.scan_topic, self.scan_callback, qos_scan)
        self.sub_path = self.create_subscription(Path, self.path_topic, self.path_callback, qos_path)
        self.sub_cmd = self.create_subscription(Twist, self.cmd_vel_topic, self.cmd_vel_callback, qos_cmd)

        timer_period_s = 1.0 / max(1e-6, self.control_hz)
        self.timer = self.create_timer(timer_period_s, self._on_control_tick)

        mode_text = "TRAIN" if self.is_training else "EVAL"
        self.get_logger().info(f"PPOTrainer indul. mode={mode_text}")
        self.get_logger().info(f"Run mappa: {self.run_dir}")
        self.get_logger().info("1 launch = 1 epizód, epizód végén shutdown.")
        self.get_logger().info(f"MPPI node: {self.controller_server_node}")
        self.get_logger().info(f"Topics: odom={self.odom_topic} scan={self.scan_topic} path={self.path_topic} cmd={self.cmd_vel_topic}")


    def odom_callback(self, msg):
        """Odom üzenet cache-elése."""
        
        self.latest_odom = msg

    def scan_callback(self, msg):
        """LaserScan üzenet cache-elése."""
        
        self.latest_scan = msg

    def path_callback(self, msg):
        """Path üzenet cache-elése."""
        
        self.latest_path = msg

    def cmd_vel_callback(self, msg):
        """cmd_vel üzenet cache-elése."""
        
        self.latest_cmd_vel = msg


    def _init_csv_if_missing(self, csv_path, header):
        """Létrehozzza a CSV-t fejléccel, ha még nem létezik"""
        
        if os.path.exists(csv_path):
            return
        with open(csv_path, "w", newline="") as f:
            csv.writer(f).writerow(header)

    def _append_csv_row(self, csv_path, row):
        """Hozzáfűz egy sort a CSV-hez"""
        
        with open(csv_path, "a", newline="") as f:
            csv.writer(f).writerow(row)

    def _read_last_csv_row_as_dict(self, csv_path):
        """Beolvassa a CSV utolsó sorát dictinonary-ként a header alapján."""
        
        if not os.path.exists(csv_path):
            return None
        
        with open(csv_path, "r", newline="") as f:
            rows = list(csv.reader(f))
            
        if len(rows) < 2:
            return None

        header = rows[0]
        last = rows[-1]
        if len(last) != len(header):
            return None

        out = {}
        for i in range(len(header)):
            out[header[i]] = last[i]
            
        return out

    def _load_best_model_if_exists(self):
        """Betölti a best_latest.pth modellt, ha van."""
        
        if not os.path.exists(self.best_model_path):
            self.get_logger().warn("best_latest.pth nincs így nulláról indulunk.")
            return

        try:
            self.ppo_trainer.policy.save_file = self.best_model_path
            self.ppo_trainer.policy.load_from_file()
            self.ppo_trainer.copy_policy()
            self.get_logger().info(f"best_latest.pth betöltve: {self.best_model_path}")
            
        except Exception as e:
            self.get_logger().error(f"best_latest.pth betöltésénél hiab van: {e}")


    def _on_control_tick(self):
        """Egy vezérlési lépés: state-reward-store, done esetén mentés és shutdownoljuk."""
        
        if self.latest_odom is None or self.latest_scan is None or self.latest_path is None:
            return
        
        if len(self.latest_path.poses) < 2:
            return

        # Epizód eleje: egyszer választunk actiont és beállítjuk az MPPI paramokat
        if self.step_index == 0:
            self._start_episode()

        state, info = self._build_state_and_info(self.latest_odom, self.latest_scan, self.latest_path)
        if state is None or info is None:
            return

        goal_distance_m = info["distance_goal"]
        min_range_m = info["min_range"]
        cross_track_error_m = info["cross_track_error"]

        # haladás: cél távolság csökkenése
        delta_goal_distance_m = 0.0
        if self.previous_goal_distance_m is not None:
            delta_goal_distance_m = float(self.previous_goal_distance_m - goal_distance_m)
            self.total_progress_m += delta_goal_distance_m
            
        self.previous_goal_distance_m = float(goal_distance_m)

        # energia: cmd_vel ugrások összege
        energy_step = self._compute_energy_step_from_cmd_vel()

        # stuck: ha nem csökken érdemben a távolság
        if abs(delta_goal_distance_m) < self.stuck_delta_eps:
            self.stuck_steps_count += 1
        else:
            self.stuck_steps_count = 0

        can_finish_as_goal = (self.step_index >= self.min_steps_for_goal)

        done = False
        reason = "running"
        
        near_wall_penalty = 0.0
        if min_range_m < self.near_obstacle_distance_m:
            danger = (self.near_obstacle_distance_m - min_range_m) / max(1e-6, self.near_obstacle_distance_m)
            near_wall_penalty = self.near_obstacle_weight * (danger * danger)

        if can_finish_as_goal and goal_distance_m < self.goal_tolerance_m:
            reward = 50.0
            done = True
            reason = "goal"
            
        elif min_range_m < self.collision_distance_m:
            reward = -50.0
            done = True
            reason = "collision"
            
        elif self.stuck_steps_count >= self.stuck_window_steps:
            reward = -20.0
            done = True
            reason = "stuck"
            
        elif self.step_index >= self.max_steps:
            if goal_distance_m < 2.0:
                reward = -0.01 - near_wall_penalty
                done = False
                reason = "running"
            else:
                reward = -10.0
                done = True
                reason = "timeout"        
        else:          
            reward = (2.0 * delta_goal_distance_m- 0.01 - 0.15 * cross_track_error_m - self.energy_weight * energy_step-near_wall_penalty )

        if self.is_training:
            self.ppo_trainer.store(state,self.current_action, self.current_action_logprob, float(reward), bool(done))

        self.step_index += 1

        if done:
            self._finish_episode_and_shutdown(reason, goal_distance_m, min_range_m)

        if self.step_index % 20 == 0:
            rx, ry = self._get_robot_xy_in_map(self.latest_odom)
            
            if rx is None:
                return
            
            self.get_logger().info(f"dist_goal={goal_distance_m:.3f} robot_map=({rx:.2f},{ry:.2f}) min_r={min_range_m:.2f} cte={cross_track_error_m:.2f} energy_sum={self.total_energy:.3f} stuck={self.stuck_steps_count}")


    def _start_episode(self):
        """Epizód elején kiválasztja az actiont és beállítja az MPPI paramokat."""
        
        state0, info0 = self._build_state_and_info(self.latest_odom, self.latest_scan, self.latest_path)
        
        if state0 is None or info0 is None:
            return

        state_tensor = torch.tensor(state0, dtype=torch.float32)

        with torch.no_grad():
            action_dist, _ = self.ppo_trainer.policy(state_tensor)
            action_tensor = action_dist.sample() if self.is_training else action_dist.mean
            self.current_action_logprob = float(action_dist.log_prob(action_tensor).sum(-1).item())

        action_np = action_tensor.squeeze(0).cpu().numpy().astype(np.float32)
        self.current_action = action_np

        # Action [-1,1] - param tartományok
        vx_max = self._map_action_to_range(float(action_np[0]), out_min=0.20, out_max=0.60)
        cost_weight = self._map_action_to_range(float(action_np[1]), out_min=0.50, out_max=8.00)

        self.episode_vx_max = float(vx_max)
        self.episode_cost_weight = float(cost_weight)

        self._set_mppi_parameters(vx_max=self.episode_vx_max, cost_weight=self.episode_cost_weight)

        # epizód resetek
        self.previous_goal_distance_m = float(info0["distance_goal"])
        self.total_progress_m = 0.0

        self.total_energy = 0.0
        self.previous_cmd_v = None
        self.previous_cmd_w = None

        self.stuck_steps_count = 0

        self.get_logger().info(f"[EP START] MPPI vx_max={self.episode_vx_max:.3f} CostCritic.cost_weight={self.episode_cost_weight:.3f}")

    def _map_action_to_range(self, action_value, out_min, out_max):
        """Action értéket [-1,1]-ből átmappel [out_min,out_max] tartományra."""
        
        a = float(max(-1.0, min(1.0, action_value)))
        t = (a + 1.0) * 0.5
        
        return float(out_min + t * (out_max - out_min))

    def _compute_energy_step_from_cmd_vel(self):
        """Egyszerű energia-metrika: |dv|+|dw| lépésenként cmd_vel alapján."""
        
        if self.latest_cmd_vel is None:
            return 0.0

        v = float(self.latest_cmd_vel.linear.x)
        w = float(self.latest_cmd_vel.angular.z)

        if self.previous_cmd_v is None or self.previous_cmd_w is None:
            self.previous_cmd_v = v
            self.previous_cmd_w = w
            return 0.0

        dv = abs(v - self.previous_cmd_v)
        dw = abs(w - self.previous_cmd_w)

        self.previous_cmd_v = v
        self.previous_cmd_w = w

        energy_step = float(dv + dw)
        self.total_energy += energy_step
        
        return energy_step


    def _make_double_param(self, name, value):
        """ROS2 double param üzenet létrehozása."""
        
        param = RosParameter()
        param.name = name
        param.value = ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=float(value))
        return param

    def _set_mppi_parameters(self, vx_max, cost_weight):
        """MPPI paramok beállítása a controller_server set_parameters service-en."""
        
        service_name = f"{self.controller_server_node}/set_parameters"

        if not self.mppi_set_params_client.wait_for_service(timeout_sec=0.5):
            self.get_logger().warn(f"Param service nem elérhető: {service_name}")
            return

        request = SetParameters.Request()
        request.parameters = [self._make_double_param("FollowPathMPPI.vx_max", vx_max),self._make_double_param("FollowPathMPPI.CostCritic.cost_weight", cost_weight)]

        future = self.mppi_set_params_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=0.5)

        if future.result() is None:
            self.get_logger().warn("Param set: nincs válasz (timeout / hiba).")
            return

        for i, result in enumerate(future.result().results):
            if not result.successful:
                failed_name = request.parameters[i].name
                self.get_logger().error(f"Param set FAIL: {failed_name} reason={result.reason}")


    def _finish_episode_and_shutdown(self, reason, goal_distance_m, min_range_m):
        """Epizódot lezár, ment, bestet frissít és leáll."""
        
        model_path = ""

        if self.is_training:
            self.ppo_trainer.finish_episode()
            model_path = self._find_latest_model_in_run_dir()
            
            if model_path and os.path.exists(model_path):
                shutil.copyfile(model_path, self.latest_global_model_path)

        score = self._compute_score(reason, self.total_progress_m, self.step_index)

        # run CSV
        self._append_csv_row(self.run_metrics_csv,[self.step_index,reason,f"{self.episode_vx_max:.4f}",f"{self.episode_cost_weight:.4f}",
                                                   f"{goal_distance_m:.4f}", f"{min_range_m:.4f}", f"{self.total_progress_m:.4f}", 
                                                   f"{self.total_energy:.4f}",f"{score:.4f}"])

        # global CSV
        self._append_csv_row(self.global_metrics_csv,[os.path.basename(self.run_dir), self.step_index,reason, f"{self.episode_vx_max:.4f}",
                                                      f"{self.episode_cost_weight:.4f}", f"{goal_distance_m:.4f}", f"{min_range_m:.4f}",
                                                      f"{self.total_progress_m:.4f}", f"{self.total_energy:.4f}",f"{score:.4f}",model_path])

        # best frissítés: (1) steps (2) energy (3) min_range
        if model_path and os.path.exists(model_path):
            self._maybe_update_best_model(reason=reason, steps=self.step_index, min_range_m=min_range_m, progress_m=self.total_progress_m,
                                          energy=self.total_energy,score=score, model_path=model_path)

        self.get_logger().info(f"[EP END] steps={self.step_index} reason={reason} progress={self.total_progress_m:.3f} energy={self.total_energy:.3f} score={score:.2f}")
        self.get_logger().info("Leáll (1 launch = 1 epizód).")
        rclpy.shutdown()

    def _find_latest_model_in_run_dir(self):
        """Megkeresi a run mappában a legfrissebb .pth modellt."""
        
        try:
            files = [f for f in os.listdir(self.run_dir) if f.endswith(".pth") and f != "latest.pth"]
            
            if not files:
                return ""
            
            files.sort(key=lambda fn: os.path.getmtime(os.path.join(self.run_dir, fn)))
            
            return os.path.join(self.run_dir, files[-1])
        
        except Exception:
            return ""

    def _compute_score(self, reason, progress_m, steps):
        """Összpontszám számítása egyszerű szabállyal."""
        
        if reason == "goal":
            return 1000.0 + progress_m - 0.1 * steps
        
        if reason == "collision":
            return -1000.0 + progress_m - 0.1 * steps
        
        return progress_m - 0.1 * steps

    def _maybe_update_best_model(self, reason, steps, min_range_m, progress_m, energy, score, model_path):
        """Best modell frissítés logika (steps -> energy -> min_range)."""
        
        if reason != "goal":
            return

        best_row = self._read_last_csv_row_as_dict(self.best_metrics_csv)
        if best_row is None:
            self._save_as_best(steps, min_range_m, progress_m, energy, score, model_path)
            return

        try:
            best_steps = int(best_row["lepesek_szama"])
            best_energy = float(best_row["sebessegvaltozas_energia"])
            best_min_range = float(best_row["legkozelebbi_akadaly_tavolsag"])
            
        except Exception:
            self._save_as_best(steps, min_range_m, progress_m, energy, score, model_path)
            return

        if steps < best_steps:
            self._save_as_best(steps, min_range_m, progress_m, energy, score, model_path)
            return
        
        if steps > best_steps:
            return

        if energy < best_energy:
            self._save_as_best(steps, min_range_m, progress_m, energy, score, model_path)
            return
        
        if energy > best_energy:
            return

        if min_range_m > best_min_range:
            self._save_as_best(steps, min_range_m, progress_m, energy, score, model_path)

    def _save_as_best(self, steps, min_range_m, progress_m, energy, score, model_path):
        """Elmenti a best modellt és logol a best CSV-be."""
        
        try:
            shutil.copyfile(model_path, self.best_model_path)
            self._append_csv_row(self.best_metrics_csv,[os.path.basename(self.run_dir),steps,"goal",f"{min_range_m:.4f}",f"{progress_m:.4f}",
                                                        f"{energy:.4f}",f"{score:.4f}",model_path])
            
            self.get_logger().info(f"A best frissült! steps={steps} energy={energy:.3f} min_range={min_range_m:.3f} - {self.best_model_path}")
            
        except Exception as e:
            self.get_logger().error(f"A best mentése során hiba történt: {e}")


    def _build_state_and_info(self, odom, scan, path):
        """State vektor és info értékek előállítása az odom/scan/path alapján."""
        
        robot_x, robot_y = self._get_robot_xy_in_map(odom)
        if robot_x is None:
            return None, None

        robot_v = float(odom.twist.twist.linear.x)
        robot_w = float(odom.twist.twist.angular.z)

        goal_x = float(path.poses[-1].pose.position.x)
        goal_y = float(path.poses[-1].pose.position.y)
        distance_to_goal_m = math.hypot(goal_x - robot_x, goal_y - robot_y)

        scan_ranges = np.array(scan.ranges, dtype=np.float32)
        scan_ranges = np.where(np.isfinite(scan_ranges), scan_ranges, self.lidar_max_range_m)
        scan_ranges = np.clip(scan_ranges, 0.0, self.lidar_max_range_m)

        if len(scan_ranges) == 0:
            min_range_m = float(self.lidar_max_range_m)
            lidar_bins_norm = [1.0] * self.lidar_bins
        else:
            min_range_m = float(np.min(scan_ranges))
            lidar_bins_norm = self._bin_lidar_min(scan_ranges)

        cross_track_error_m = self._compute_cross_track_error(robot_x, robot_y, path)

        # normalizálás
        norm_goal_dist = min(distance_to_goal_m / 20.0, 1.0)
        norm_cte = min(cross_track_error_m / 2.0, 1.0)
        norm_v = float(np.clip(robot_v / 1.0, -1.0, 1.0))
        norm_w = float(np.clip(robot_w / 1.5, -1.0, 1.0))
        norm_min_r = float(np.clip(min_range_m / self.lidar_max_range_m, 0.0, 1.0))

        state = np.array([norm_goal_dist, norm_v, norm_w, norm_min_r, norm_cte] + lidar_bins_norm,dtype=np.float32)

        info = {"distance_goal": float(distance_to_goal_m),"min_range": float(min_range_m),"cross_track_error": float(cross_track_error_m)}
        
        return state, info

    def _bin_lidar_min(self, scan_ranges):
        """A lidar tartományt bin-ekre bontja és bin-enként minimumot ad vissza normalizálva."""
        
        total_points = len(scan_ranges)
        points_per_bin = max(1, total_points // self.lidar_bins)

        out = []
        for bin_index in range(self.lidar_bins):
            start = bin_index * points_per_bin
            end = min(total_points, (bin_index + 1) * points_per_bin)
            
            if start >= total_points:
                min_in_bin = float(self.lidar_max_range_m)
            else:
                min_in_bin = float(np.min(scan_ranges[start:end]))
            out.append(min_in_bin / self.lidar_max_range_m)
            
        return out

    def _compute_cross_track_error(self, robot_x, robot_y, path):
        """Kiszámolja a robot legkisebb távolságát a Path pontjaihoz."""
        
        if path is None or len(path.poses) == 0:
            return 0.0

        best = 1e9
        for pose_stamped in path.poses:
            px = float(pose_stamped.pose.position.x)
            py = float(pose_stamped.pose.position.y)
            d = math.hypot(px - robot_x, py - robot_y)
            
            if d < best:
                best = d
                
        return float(best)

    def _get_robot_xy_in_map(self, odom):
        """Odom pozíciót átrakja map frame-be TF segítségével."""
        
        odom_point = PointStamped()
        odom_point.header.frame_id = odom.header.frame_id
        now = self.get_clock().now()
        odom_point.header.stamp = now.to_msg()

        odom_point.point.x = float(odom.pose.pose.position.x)
        odom_point.point.y = float(odom.pose.pose.position.y)
        odom_point.point.z = 0.0

        try:
            transform = self.tf_buffer.lookup_transform("map",odom_point.header.frame_id,now,timeout=Duration(seconds=0.2))
            map_point = do_transform_point(odom_point, transform)
            
            return float(map_point.point.x), float(map_point.point.y)
        
        except Exception as e:
            self.get_logger().error(f"TF hiba: {e}")
            return None, None


def main(args=None):
    rclpy.init(args=args)
    node = PPOTrainer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()