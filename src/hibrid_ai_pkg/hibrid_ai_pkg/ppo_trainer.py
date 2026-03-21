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
import csv
import math
import os
import shutil
from datetime import datetime
import numpy as np
import torch
from .ppo_training import PPOTraining


class PPOTrainer(Node):
    """PPO tréner node, ami epizódonként MPPI paramétereket állít és metrikákat ment."""

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
        
        self.declare_parameter("lidar_sector", 12)
        self.lidar_sector = int(self.get_parameter("lidar_sector").value)
        
        self.declare_parameter("lidar_max_range", 6.0)
        self.lidar_max_range_m = float(self.get_parameter("lidar_max_range").value)
        
        self.declare_parameter("near_obstacle_distance", 0.7)   #0.6 m
        self.near_obstacle_distance_m = float(self.get_parameter("near_obstacle_distance").value)
        
        self.declare_parameter("near_obstacle_weight", 3.0) #2.0
        self.near_obstacle_weight = float(self.get_parameter("near_obstacle_weight").value)

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

        self.init_csv(self.run_metrics_csv,header=["lepesek_szama","befejezes_oka","mppi_vx_max","mppi_wz_max","mppi_vx_std","mppi_wz_std","mppi_cost_weight",
                                                                "celtol_valo_tavolsag (m)","legkozelebbi_akadaly_tavolsag",
                                                                "ossz_haladas","sebessegvaltozas_energia","kapott_pontszam"])

        self.init_csv(self.global_metrics_csv,header=["futas_azonosito","lepesek_szama","befejezes_oka","mppi_vx_max","mppi_wz_max",
                                                                  "mppi_vx_std","mppi_wz_std","mppi_cost_weight","celtol_valo_tavolsag (m)",
                                                                  "legkozelebbi_akadaly_tavolsag","ossz_haladas","sebessegvaltozas_energia","kapott_pontszam","modell_fajl"])

        self.init_csv(self.best_metrics_csv,header=["futas_azonosito","lepesek_szama","befejezes_oka","legkozelebbi_akadaly_tavolsag",
                                                                "ossz_haladas","sebessegvaltozas_energia","kapott_pontszam","forras_modell"])

        # cache
        self.latest_odom = None
        self.latest_scan = None
        self.latest_path = None
        self.latest_cmd_vel = None

        # Epizód állapot
        self.step_index = 0
        self.previous_goal_distance_m = None
        self.total_progress_m = 0.0

        self.total_energy = 0.0
        self.previous_cmd_linear_x = None
        self.previous_cmd_angular_z = None

        self.stuck_steps_count = 0

        # PPO action + logprob
        self.current_action = np.array([0.0, 0.0, 0.0, 0.0, 0.0], dtype=np.float32)
        self.current_action_logprob = 0.0

        # MPPI paraméterek
        self.episode_vx_max = 0.0
        self.episode_wz_max = 0.0
        self.episode_vx_std = 0.0
        self.episode_wz_std = 0.0
        self.episode_cost_weight = 0.0

        # PPO tréner
        self.state_dim = 5 + self.lidar_sector
        self.action_dim = 5
        self.ppo_trainer = PPOTraining(state_dim=self.state_dim, action_dim=self.action_dim)
        self.ppo_trainer.save_dir = self.run_dir
        os.makedirs(self.ppo_trainer.save_dir, exist_ok=True)
        self.ppo_trainer.save_gyakorisag = 1  # 1 epizód = 1 mentés

        if self.is_training:
            self.load_best_model_if_exists()

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.mppi_set_params_client = self.create_client(SetParameters, f"{self.controller_server_node}/set_parameters")
  
        qos_path = QoSProfile(depth=10)
        qos_path.reliability = ReliabilityPolicy.RELIABLE
        qos_path.durability = DurabilityPolicy.TRANSIENT_LOCAL
        
        self.sub_path = self.create_subscription(Path, self.path_topic, self.path_callback, qos_path)
        
        qos_cmd = QoSProfile(depth=20)
        qos_cmd.reliability = ReliabilityPolicy.RELIABLE
        qos_cmd.durability = DurabilityPolicy.VOLATILE
        
        self.sub_cmd = self.create_subscription(Twist, self.cmd_vel_topic, self.cmd_vel_callback, qos_cmd)
        
        qos_scan = QoSProfile(depth=10)
        qos_scan.reliability = ReliabilityPolicy.RELIABLE
        qos_scan.durability = DurabilityPolicy.VOLATILE
        
        self.sub_scan = self.create_subscription(LaserScan, self.scan_topic, self.scan_callback, qos_scan)
        
        qos_odom = QoSProfile(depth=20)
        qos_odom.reliability = ReliabilityPolicy.RELIABLE
        qos_odom.durability = DurabilityPolicy.VOLATILE

        self.sub_odom = self.create_subscription(Odometry, self.odom_topic, self.odom_callback, qos_odom)
        
        if self.control_hz <= 0:
            self.get_logger().error("Hibás control_hz!!!!! Alapértelmezett 10 Hz lesz!!!!")
            effective_hz = 10.0
        else:
            effective_hz = self.control_hz

        timer_period_s = 1.0 / effective_hz
        self.timer = self.create_timer(timer_period_s, self.on_control_tick)

        mode_text = "TRAIN" if self.is_training else "EVAL"
        self.get_logger().debug(f"PPOTrainer indul. mode={mode_text}")
        self.get_logger().info(f"Run mappa: {self.run_dir}")
        self.get_logger().info("1 launch = 1 epizód, epizód végén shutdown.")
        self.get_logger().info(f"MPPI node: {self.controller_server_node}")
        self.get_logger().info(f"Topics: odom={self.odom_topic} scan={self.scan_topic} path={self.path_topic} cmd={self.cmd_vel_topic}")


    # elcacheljük az odom üzenetet
    def odom_callback(self, msg):     
        self.latest_odom = msg

    ## elcacheljük a scan üzenetet
    def scan_callback(self, msg):
        self.latest_scan = msg

    # elcacheljük a path üzenetet
    def path_callback(self, msg):
        self.latest_path = msg

    # elcacheljük a cmd_vel üzenetet
    def cmd_vel_callback(self, msg):      
        self.latest_cmd_vel = msg


    #Létrehozzza a CSV-t fejléccel, ha még nem létezik...
    def init_csv(self, csv_path, header):       
        if os.path.exists(csv_path):
            return
        
        with open(csv_path, "w", newline="") as f:
            csv.writer(f).writerow(header)


    #Hozzáfűz egy sort a CSV-hez....
    def append_csv_row(self, csv_path, row):
        
        with open(csv_path, "a", newline="") as f:
            csv.writer(f).writerow(row)

    #Beolvassa a CSV utolsó sorát dictinonary-ként a header alapján...
    def read_last_csv_row_in_dictionary(self, csv_path):        
        if not os.path.exists(csv_path):
            self.get_logger().error("Nem létezik a fájl!!!!")
            return None
        
        with open(csv_path, "r", newline="") as f:
            rows = list(csv.reader(f))
            
        if len(rows) < 2:
            self.get_logger().error("Csak a header van és üüres a fájl!!!!")
            return None

        header = rows[0]
        last_row = rows[-1]
        
        if len(last_row) != len(header):
            self.get_logger().error("A féjlban az utolsó sor sérült!!!!")
            return None

        last_csv_row = {}
        for i in range(len(header)):
            last_csv_row[header[i]] = last_row[i]
            
        return last_csv_row

    # Betölti a best_latest.pth modellt, ha van...
    def load_best_model_if_exists(self):   
        if not os.path.exists(self.best_model_path):
            self.get_logger().warn("best_latest.pth nincs így nulláról indulunk!!!")
            return

        try:
            self.ppo_trainer.policy.save_file = self.best_model_path
            self.ppo_trainer.policy.load_from_file()
            self.ppo_trainer.copy_policy()
            self.get_logger().debug(f"best_latest.pth betöltve: {self.best_model_path}")
            
        except Exception as e:
            self.get_logger().error(f"best_latest.pth betöltésénél hiab van!!!!!!!!! : {e}")


    # vezérlési cilus: state-reward-store-done esetén mentés és shutdownoljuk...
    def on_control_tick(self):     
        if self.latest_odom is None:
            self.get_logger().warn("Várakozás odom-ra...")
            return
            
        if self.latest_scan is None:
            self.get_logger().warn("Várakozás scan-re...")
            return
            
        if self.latest_path is None:
            self.get_logger().warn("Várakozás path-ra...")
            return
         
        if len(self.latest_path.poses) < 2:
            self.get_logger().warn(f"Path túl rövid: {len(self.latest_path.poses)} pose < 2")
            return
        
        self.get_logger().info("Minden input rendben mgjött!!!")

        # Epizód eleje egyszer választunk actiont és beállítjuk az MPPI parammétereket
        if self.step_index == 0:
            self.start_episode()

        state, info = self.build_state_and_info(self.latest_odom, self.latest_scan, self.latest_path)
        if state is None or info is None:
            self.get_logger().error("build_state_and_info() None-t adott vissza!!! State építés nem sikerült!!!")
            return

        goal_distance_m = info["distance_goal"]
        min_range_m = info["min_range"]
        cross_track_error_m = info["cross_track_error"]

        # haladás: cél távolság csökkenése
        delta_goal_distance_m = 0.0
        if self.previous_goal_distance_m is not None:
            delta_goal_distance_m = float(self.previous_goal_distance_m - goal_distance_m)
            self.total_progress_m = self.total_progress_m + delta_goal_distance_m
            
        self.previous_goal_distance_m = float(goal_distance_m)

        # energia: cmd_vel ugrások összege
        energy_step = self.compute_energy_step_from_cmd_vel()

        # stuck van, ha nem csökken érdemben a távolság
        if abs(delta_goal_distance_m) < self.stuck_delta_eps:
            self.stuck_steps_count = self.stuck_steps_count+1
        else:
            self.stuck_steps_count = 0

        enough_steps_passed = (self.step_index >= self.min_steps_for_goal)
        close_enough_to_goal = (goal_distance_m < self.goal_tolerance_m)

        done = False
        reason = "running"
        
        near_wall_penalty = 0.0
        if min_range_m < self.near_obstacle_distance_m:
            danger = (self.near_obstacle_distance_m - min_range_m) / max(1e-6, self.near_obstacle_distance_m)
            near_wall_penalty = self.near_obstacle_weight * (danger * danger)

        if enough_steps_passed and close_enough_to_goal:
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
            # reward = (2.0 * delta_goal_distance_m- 0.01 - 0.15 * cross_track_error_m - self.energy_weight * energy_step-near_wall_penalty )
            reward = (2.0 * delta_goal_distance_m- 0.005 - 0.25 * cross_track_error_m - self.energy_weight * energy_step-near_wall_penalty )

        if self.is_training:
            self.ppo_trainer.store(state,self.current_action, self.current_action_logprob, float(reward), bool(done))

        self.step_index = self.step_index + 1

        if done:
            self.finish_episode_and_shutdown(reason, goal_distance_m, min_range_m)

        if self.step_index % 20 == 0:
            rbot_x, robot_y = self.get_robot_pose_from_odom_in_map(self.latest_odom)
            
            if rbot_x is None:
                self.get_logger().warn("Robot pozíciója None!!! TF/odom hiba!!!!")
                return
            
            self.get_logger().info(f"distance_goal={goal_distance_m:.3f} robot_map=({rbot_x:.2f},{robot_y:.2f}) min_range={min_range_m:.2f}" 
                                   f"cross_track_error={cross_track_error_m:.2f} energy_sum={self.total_energy:.3f} stuck={self.stuck_steps_count}")


    #Epizó elején kiválasztja az actoint és beállítja az MPPI parametereket...
    def start_episode(self):   
        state, info = self.build_state_and_info(self.latest_odom, self.latest_scan, self.latest_path)
        
        if state is None or info is None:
            self.get_logger().error("Epizód start hiba: nincs odom/scan/path!!!!!")
            return

        state_tensor = torch.tensor(state, dtype=torch.float32)

        with torch.no_grad():
            action_distribution, _ = self.ppo_trainer.policy(state_tensor)
            
            if self.is_training:
                action_tensor = action_distribution.sample() 
            else:
                action_tensor = action_distribution.mean 
            
            self.current_action_logprob = float(action_distribution.log_prob(action_tensor).sum(-1).item())

        action_array = action_tensor.squeeze(0).cpu().numpy().astype(np.float32)
        self.current_action = action_array

        VX_MAX_OUT_MIN = 0.20
        VX_MAX_OUT_MAX = 0.65
        WZ_MAX_OUT_MIN = 1.00
        WZ_MAX_OUT_MAX = 2.50    
        VX_STD_OUT_MIN =0.06
        VX_STD_OUT_MAX=0.20
        WZ_STD_OUT_MIN = 0.20
        WZ_STD_OUT_MAX = 0.45
        COST_WEIGHT_MIN = 0.50
        COST_WEIGHT_MAX = 8.00
           
        vx_max = self.map_action_to_range(float(action_array[0]), VX_MAX_OUT_MIN, VX_MAX_OUT_MAX)
        wz_max = self.map_action_to_range(float(action_array[1]), WZ_MAX_OUT_MIN, WZ_MAX_OUT_MAX )
        vx_std = self.map_action_to_range(float(action_array[2]), VX_STD_OUT_MIN, VX_STD_OUT_MAX)
        wz_std = self.map_action_to_range(float(action_array[3]), WZ_STD_OUT_MIN, WZ_STD_OUT_MAX )
        cost_weight = self.map_action_to_range(float(action_array[4]), COST_WEIGHT_MIN, COST_WEIGHT_MAX)
             
        self.episode_vx_max = float(vx_max)
        self.episode_wz_max = float(wz_max)
        self.episode_vx_std = float(vx_std)
        self.episode_wz_std = float(wz_std)
        self.episode_cost_weight = float(cost_weight)

        self.set_mppi_parameters(self.episode_vx_max,self.episode_wz_max,self.episode_vx_std,self.episode_wz_std,self.episode_cost_weight)

        self.previous_goal_distance_m = float(info["distance_goal"])
        self.total_progress_m = 0.0
        self.total_energy = 0.0
        self.previous_cmd_linear_x = None
        self.previous_cmd_angular_z = None
        self.stuck_steps_count = 0

        self.get_logger().debug(f"Epizód start!!! MPPI vx_max={self.episode_vx_max:.3f} wz_max={self.episode_wz_max:.3f} vx_std={self.episode_vx_std:.3f}"
                               f"wz_std={self.episode_wz_std:.3f} CostCritic.cost_weight={self.episode_cost_weight:.3f}")

    # Action értéket [-1,1]-ből átmappel [out_min,out_max] tartományra...
    def map_action_to_range(self, action_value, out_min, out_max):
        safe_action = float(max(-1.0, min(1.0, action_value))) # np.clip(action_value, -1, 1)
        normalized = (safe_action + 1.0) * 0.5
        
        return float(out_min + normalized * (out_max - out_min))


    #energa-metrika: |delta_v|+|delta_w| ,lépésenként cmdvel alapján...
    def compute_energy_step_from_cmd_vel(self):
        if self.latest_cmd_vel is None:
            return 0.0

        actual_linear_x = float(self.latest_cmd_vel.linear.x)
        actual_angular_z = float(self.latest_cmd_vel.angular.z)

        if self.previous_cmd_linear_x is None or self.previous_cmd_angular_z is None:
            self.previous_cmd_linear_x = actual_linear_x
            self.previous_cmd_angular_z = actual_angular_z
            return 0.0

        delta_v = abs(actual_linear_x - self.previous_cmd_linear_x)
        delta_w = abs(actual_angular_z - self.previous_cmd_angular_z)

        self.previous_cmd_linear_x = actual_linear_x
        self.previous_cmd_angular_z = actual_angular_z

        energy_step = float(delta_v + delta_w)
        self.total_energy =self.total_energy + energy_step
        
        return energy_step

    '''
    #MPPI paramok beállítása a controller_server set_parameters service-en...
    def set_mppi_parameters(self, vx_max, wz_max, vx_std, wz_std, cost_weight):      
        service_name = f"{self.controller_server_node}/set_parameters"
		
        mppi_paramters = [("FollowPathMPPI.vx_max", vx_max), ("FollowPathMPPI.wz_max", wz_max), ("FollowPathMPPI.vx_std", vx_std), 
                          ("FollowPathMPPI.wz_std", wz_std), ("FollowPathMPPI.CostCritic.cost_weight", cost_weight)]

        if not self.mppi_set_params_client.wait_for_service(timeout_sec=0.5):
            self.get_logger().error(f"Service nem elérhető: {service_name} !!!!!!")
            return

        request = SetParameters.Request()
        request.parameters = []
		
		for name, value in mppi_paramters:
		    param = RosParameter()
			param.name = name
			param.value = ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=float(value))
			
			request.parameters.append(param)
			

        future = self.mppi_set_params_client.call_async(request)
        #rclpy.spin_until_future_complete(self, future, timeout_sec=0.8) # EZ LEHET GONDOT FOG OKOZNI!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

        if future.result() is None:
            self.get_logger().error("Timeout hiba a paraméterküldésnél!!!!!!!!!")
            return

        for i, result in enumerate(future.result().results):
            if not result.successful:
                self.get_logger().error(f"Sikertelen: {request.parameters[i].name} reason={result.reason} !!!!!!!")

    '''
    # MPPI paramok beállítása a controller_server set_parameters service-en...
    def set_mppi_parameters(self, vx_max, wz_max, vx_std, wz_std, cost_weight):
        service_name = f"{self.controller_server_node}/set_parameters"

        mppi_paramters = [("FollowPathMPPI.vx_max", vx_max),("FollowPathMPPI.wz_max", wz_max), ("FollowPathMPPI.vx_std", vx_std),
                           ("FollowPathMPPI.wz_std", wz_std),("FollowPathMPPI.CostCritic.cost_weight", cost_weight)]

        if not self.mppi_set_params_client.wait_for_service(timeout_sec=0.5):
            self.get_logger().error(f"Service nem elérhető: {service_name} !!!!!!")
            return

        request = SetParameters.Request()
        request.parameters = []

        for name, value in mppi_paramters:
            param = RosParameter()
            param.name = name
            param.value = ParameterValue(type=ParameterType.PARAMETER_DOUBLE,double_value=float(value))
            
            request.parameters.append(param)

        future = self.mppi_set_params_client.call_async(request)
        # rclpy.spin_until_future_complete(self, future, timeout_sec=0.8)

        if future.result() is None:
            self.get_logger().error("Timeout hiba a paraméterküldésnél!!!!!!!!!")
            return

        for i, result in enumerate(future.result().results):
            if not result.successful:
                self.get_logger().error(f"Sikertelen: {request.parameters[i].name} reason={result.reason} !!!!!!!")

    #Epizódot lezár, ment, bestet frissít és leáll...
    def finish_episode_and_shutdown(self, reason, goal_distance_m, min_range_m):     
        model_path = ""
        is_model_path = False

        if self.is_training:
            self.ppo_trainer.finish_episode()
            model_path = self.find_latest_model_in_run_dir()
            
            is_model_path = model_path and os.path.exists(model_path)
            
            if is_model_path:
                shutil.copyfile(model_path, self.latest_global_model_path)

        score = self.compute_score(reason, self.total_progress_m, self.step_index)
        
        metrics_row = [self.step_index,reason,f"{self.episode_vx_max:.4f}",  f"{self.episode_wz_max:.4f}", f"{self.episode_vx_std:.4f}",
                       f"{self.episode_wz_std:.4f}",f"{self.episode_cost_weight:.4f}",f"{goal_distance_m:.4f}", f"{min_range_m:.4f}",
                       f"{self.total_progress_m:.4f}",f"{self.total_energy:.4f}",f"{score:.4f}"]

        global_row = [os.path.basename(self.run_dir)] + metrics_row + [model_path]

        self.append_csv_row(self.run_metrics_csv, metrics_row)
    
        self.append_csv_row(self.global_metrics_csv, global_row)

        if is_model_path:
            self.update_best_model(reason,self.step_index, min_range_m,self.total_progress_m, self.total_energy, score, model_path)

        self.get_logger().info(f"Epizód vége!!! Lépések={self.step_index} Ok={reason} Előrehaladás={self.total_progress_m:.3f} Energia={self.total_energy:.3f}" f"Pontszám={score:.2f}")
        self.get_logger().info("Leáll (1 launch = 1 epizód)....")
        rclpy.shutdown()


    #Megkeresi a run mappában a legfrissebb .pth modellt...
    def find_latest_model_in_run_dir(self):
        try:
            all_files = os.listdir(self.run_dir)
            path_files = []
            
            for file in all_files:
                if file.endswith(".pth") and file != "latest.pth":
                    path_files.append(file)
            
            if not path_files:
                return ""
            
            file_paths = []
            
            for filename in path_files:
                full_path = os.path.join(self.run_dir, filename)
                mod_time = os.path.getmtime(full_path)
                file_paths.append((mod_time, full_path))
            
            file_paths.sort()
            
            return file_paths[-1][1]  # csak az útvonalat adjuk vissza
        
        except Exception as e:
            self.get_logger().error(f"A modell keresese soán hiba történét: {e} !!!!!")
            return ""


    #Összpontszám számítása egyszerű szabállyal...
    def compute_score(self, reason, progress_m, steps):
        basic = progress_m - 0.1 * steps 
        
        if reason == "goal":
            return 1000.0 + basic
        
        if reason == "collision":
            return -1000.0 + basic
        
        return basic

    #Best modell frissítés logika steps - energy - min_range...
    def update_best_model(self, reason, steps, min_range_m, progress_m, energy, score, model_path):     
        if reason != "goal":
            return

        best_row = self.read_last_csv_row_in_dictionary(self.best_metrics_csv)
        if best_row is None:
            self.save_best(steps, min_range_m, progress_m, energy, score, model_path)
            return

        try:
            best_steps = int(best_row["lepesek_szama"])
            best_energy = float(best_row["sebessegvaltozas_energia"])
            best_min_range = float(best_row["legkozelebbi_akadaly_tavolsag"])
            
        except Exception:
            self.save_best(steps, min_range_m, progress_m, energy, score, model_path)
            return

        if steps < best_steps:
            self.save_best(steps, min_range_m, progress_m, energy, score, model_path)
            return
        
        if steps > best_steps:
            return

        if energy < best_energy:
            self.save_best(steps, min_range_m, progress_m, energy, score, model_path)
            return
        
        if energy > best_energy:
            return

        if min_range_m > best_min_range:
            self.save_best(steps, min_range_m, progress_m, energy, score, model_path)
            

    #Elmenti a best modellt a best CSV-be...
    def save_best(self, steps, min_range_m, progress_m, energy, score, model_path): 
        try:
            shutil.copyfile(model_path, self.best_model_path)
            self.append_csv_row(self.best_metrics_csv,[os.path.basename(self.run_dir),steps,"goal",f"{min_range_m:.4f}",f"{progress_m:.4f}",
                                                        f"{energy:.4f}",f"{score:.4f}",model_path])
            
            self.get_logger().debug(f"A best frissült! steps={steps} energy={energy:.3f} min_range={min_range_m:.3f} - {self.best_model_path}")
            
        except Exception as e:
            self.get_logger().error(f"A best mentése során hiba történt: {e} !!!!!!")
            

    #State vektor és info értékek előállíátsa az odom/scan/path alapján...
    def build_state_and_info(self, odom, scan, path):  
        robot_map_x, robot_map_y = self.get_robot_pose_from_odom_in_map(odom)
        if robot_map_x is None:
            return None, None

        robot_linear_speed = float(odom.twist.twist.linear.x)
        robot_angular_speed = float(odom.twist.twist.angular.z)

        goal_map_x = float(path.poses[-1].pose.position.x)
        goal_map_y = float(path.poses[-1].pose.position.y)
        goal_distance_m = math.hypot(goal_map_x - robot_map_x, goal_map_y - robot_map_y)

        cleand_ranges = []
        max_range = self.lidar_max_range_m
        
        for i in scan.ranges:
            if 0 < i <= max_range and not math.isnan(i) and math.isfinite(i):
                cleand_ranges.append(i)
            else:
                cleand_ranges.append(max_range)
        
        if cleand_ranges:
            min_range_m = min(cleand_ranges)
            normalized_lidar_sector = self.lidar_sector_min_distances(cleand_ranges)  # eredeti függvény marad
        else:
            min_range_m = max_range
            normalized_lidar_sector = [1.0] * self.lidar_sector

        cross_track_error_m = self.compute_cross_track_error(robot_map_x, robot_map_y, path)

        # normalizálás
        normalized_goal_distance  = min(goal_distance_m / 20.0, 1.0)
        normalized_cross_track_error  = min(cross_track_error_m / 2.0, 1.0)
        normalized_linear_velocity  = max(min(robot_linear_speed / 1.0, 1.0), -1.0)
        normalized_angular_velocity  = max(min(robot_angular_speed / 1.5, 1.0), -1.0)
        normalized_min_lidar_range  = min_range_m / max_range

        state = [normalized_goal_distance , normalized_linear_velocity , normalized_angular_velocity , normalized_min_lidar_range , normalized_cross_track_error ] + normalized_lidar_sector

        info = {"distance_goal": goal_distance_m, "min_range": min_range_m,"cross_track_error": cross_track_error_m}
        
        return state, info


    #A lidar tartományt szektorokra bontja és szektoronként a minimumot adja vissza normalizálva...
    def lidar_sector_min_distances(self, scan_ranges):    
        total_points = len(scan_ranges)
        points_per_sector = max(1, total_points // self.lidar_sector)

        normalized_lidar_sector_result = []
        for i in range(self.lidar_sector):
            sector_start = i * points_per_sector
            sector_end = min(total_points, (i + 1) * points_per_sector)
            
            if sector_start >= total_points:
                minimum_distance = self.lidar_max_range_m
            else:
                minimum_distance = np.min(scan_ranges[sector_start:sector_end])
                
            normalized_lidar_sector_result.append(minimum_distance / self.lidar_max_range_m)
            
        return normalized_lidar_sector_result

    
    #Kiszámolja a robot legkisebbb távolságát a Path pontjaihoz...
    def compute_cross_track_error(self, robot_x, robot_y, path, radius=5.0):     
        if path is None or len(path.poses) == 0:
            return 0.0

        best_min_distance = float('inf')
        
        for pose_stamped in path.poses:
            path_x = pose_stamped.pose.position.x
            path_y = pose_stamped.pose.position.y        
            delta_x = path_x - robot_x
            delta_y = path_y - robot_y
            
            if delta_x*delta_x + delta_y*delta_y > radius*radius:
                continue
            
            euclides_distance = math.hypot(delta_x, delta_y)
            best_min_distance = min(best_min_distance, euclides_distance)
             
        return best_min_distance
    
    
    #Odom pozíciót átalakítjamap frame-be TF segítségével...
    def get_robot_pose_from_odom_in_map(self, odom):     
        odom_point = PointStamped()
        odom_point.header.frame_id = odom.header.frame_id
        #now = self.get_clock().now()
        #odom_point.header.stamp = self.get_clock().now().to_msg()
        odom_point.header.stamp = odom.header.stamp

        odom_point.point.x = float(odom.pose.pose.position.x)
        odom_point.point.y = float(odom.pose.pose.position.y)
        odom_point.point.z = 0.0

        try:
            transform = self.tf_buffer.lookup_transform("map",odom_point.header.frame_id,odom_point.header.stamp,timeout=Duration(seconds=0.2))
            map_point = do_transform_point(odom_point, transform)
            
            return map_point.point.x, map_point.point.y
        
        except Exception as e:
            self.get_logger().error(f"TF hiba van: {e} !!!!!")
            return None, None


def main(args=None):
    rclpy.init(args=args)
    node = PPOTrainer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()