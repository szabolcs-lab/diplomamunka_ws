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
import math
import os
import numpy as np
import torch
from .ppo_training import PPOTraining


class PPOProduct(Node):
    """PPO product node: betölti a best_latest.pth-t és egyszer beállítja az MPPI paramokat..."""

    def __init__(self):
        super().__init__("ppo_product")

        self.declare_parameter("control_hz", 10.0)
        self.control_hz = float(self.get_parameter("control_hz").value)

        self.declare_parameter("lidar_sector", 12)
        self.lidar_sector = int(self.get_parameter("lidar_sector").value)

        self.declare_parameter("lidar_max_range", 6.0)
        self.lidar_max_range_m = float(self.get_parameter("lidar_max_range").value)

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

        self.declare_parameter("runs_dir", "./ppo_runs")
        self.runs_dir = str(self.get_parameter("runs_dir").value)
        os.makedirs(self.runs_dir, exist_ok=True)

        self.best_model_path = os.path.join(self.runs_dir, "best_latest.pth")

        self.latest_odom = None
        self.latest_scan = None
        self.latest_path = None
        self.latest_cmd_vel = None
        self.step_index = 0
        self.set_once = False

        self.state_dim = 5 + self.lidar_sector
        self.action_dim = 5
        self.ppo_trainer = PPOTraining(state_dim=self.state_dim, action_dim=self.action_dim)

        self.load_best_model_if_exists()
  
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.mppi_set_params_client = self.create_client(SetParameters,f"{self.controller_server_node}/set_parameters" )

        self.last_setparams_request = None
        self.last_setparams_info = None
        self.last_setparams_values = None

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
        self.timer = self.create_timer(timer_period_sec, self.on_control_tick)

        self.get_logger().info("PPOProduct indul. Egyszeri MPPI param beállítás...")
        self.get_logger().info(f"MPPI node: {self.controller_server_node}")
        self.get_logger().info(f"Topics: odom={self.odom_topic} scan={self.scan_topic} path={self.path_topic} cmd={self.cmd_vel_topic}")


    def odom_callback(self, msg):
        self.latest_odom = msg

    def scan_callback(self, msg):
        self.latest_scan = msg

    def path_callback(self, msg):
        self.latest_path = msg

    def cmd_vel_callback(self, msg):
        self.latest_cmd_vel = msg


    def load_best_model_if_exists(self):
        if not os.path.exists(self.best_model_path):
            self.get_logger().error(f"best_latest.pth nincs itt: {self.best_model_path}")
            return

        try:
            self.ppo_trainer.policy.save_file = self.best_model_path
            self.ppo_trainer.policy.load_from_file()
            self.ppo_trainer.copy_policy()
            self.get_logger().info(f"best_latest.pth betöltve: {self.best_model_path}")
        except Exception as e:
            self.get_logger().error(f"best_latest.pth betöltés hiba: {e} !!!!!!!!")
            raise


    def on_control_tick(self):
        if self.set_once:
            return

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
            self.get_logger().warn(f"Path túl rövid: {len(self.latest_path.poses)} pose < 2 !")
            return


        service_name = f"{self.controller_server_node}/set_parameters"
        if not self.mppi_set_params_client.service_is_ready():
            self.get_logger().warn(f"Várakozás service-re: {service_name} .....")
            return

        self.start_episode()


    def start_episode(self):
        state, info = self.build_state_and_info(self.latest_odom, self.latest_scan, self.latest_path)
        if state is None or info is None:
            self.get_logger().error("Epizód start hiba: state/info None!!!!!")
            return

        state_tensor = torch.tensor(state, dtype=torch.float32).unsqueeze(0)

        with torch.no_grad():
            action_distribution, _ = self.ppo_trainer.policy(state_tensor)
            action_tensor = action_distribution.mean  #mean

        action_array = action_tensor.squeeze(0).cpu().numpy().astype(np.float32)

        VX_MAX_OUT_MIN = 0.20
        VX_MAX_OUT_MAX = 0.65
        WZ_MAX_OUT_MIN = 1.00
        WZ_MAX_OUT_MAX = 2.50
        VX_STD_OUT_MIN = 0.06
        VX_STD_OUT_MAX = 0.20
        WZ_STD_OUT_MIN = 0.20
        WZ_STD_OUT_MAX = 0.45
        COST_WEIGHT_MIN = 0.50
        COST_WEIGHT_MAX = 8.00

        vx_max = self.map_action_to_range(float(action_array[0]), VX_MAX_OUT_MIN, VX_MAX_OUT_MAX)
        wz_max = self.map_action_to_range(float(action_array[1]), WZ_MAX_OUT_MIN, WZ_MAX_OUT_MAX)
        vx_std = self.map_action_to_range(float(action_array[2]), VX_STD_OUT_MIN, VX_STD_OUT_MAX)
        wz_std = self.map_action_to_range(float(action_array[3]), WZ_STD_OUT_MIN, WZ_STD_OUT_MAX)
        cost_weight = self.map_action_to_range(float(action_array[4]), COST_WEIGHT_MIN, COST_WEIGHT_MAX)


        self.set_mppi_parameters(vx_max, wz_max, vx_std,wz_std, cost_weight, info)
        self.set_once = True 

 
    def map_action_to_range(self, action_value, out_min, out_max):
        safe_action = float(max(-1.0, min(1.0, action_value)))
        normalized = (safe_action + 1.0) * 0.5
        
        return float(out_min + normalized * (out_max - out_min))

 
    def set_mppi_parameters(self, vx_max, wz_max, vx_std, wz_std, cost_weight, info):
        mppi_paramters = [("FollowPathMPPI.vx_max", vx_max), ("FollowPathMPPI.wz_max", wz_max), ("FollowPathMPPI.vx_std", vx_std),
                          ("FollowPathMPPI.wz_std", wz_std), ("FollowPathMPPI.CostCritic.cost_weight", cost_weight)]
        
        request = SetParameters.Request()
        request.parameters = []
        
        for name, value in mppi_paramters:
			param = RosParameter()
			param.name = name
			param.value = ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=float(value))
			
			request.parameters.append(param)

        # eltesszük, hogy a callback tudjon logolni (nálad úgyis egyszer fut)
        self.last_setparams_request = request
        self.last_setparams_info = info
        self.last_setparams_values = (vx_max, wz_max, vx_std, wz_std, cost_weight)

        future = self.mppi_set_params_client.call_async(request)
        future.add_done_callback(self.on_set_mppi_parameters_done)

        self.get_logger().info("MPPI paramok elküldve (async)....")
        

    def on_set_mppi_parameters_done(self, fut):
        try:
            res = fut.result()
            if res is None:
                self.get_logger().error("set_parameters: nincs válasz (None)!!!!!")
                return

            request = self.last_setparams_request
            info = self.last_setparams_info
            vx_max, wz_max, vx_std, wz_std, cost_weight = self.last_setparams_values

            for i, result in enumerate(res.results):
                if not result.successful:
                    self.get_logger().error(f"Sikertelen: {request.parameters[i].name} reason={result.reason} !!!!!!")
                    return

            self.get_logger().warn(f"MPPI paramok beállítva OK: vx_max={vx_max:.3f} wz_max={wz_max:.3f} vx_std={vx_std:.3f} wz_std={wz_std:.3f} cost_w={cost_weight:.3f} | "
                                   f"dist_goal={info['distance_goal']:.2f} min_range={info['min_range']:.2f}")

        except Exception as e:
            self.get_logger().error(f"set_parameters callback exception: {e} !!!!!!")

    
    def build_state_and_info(self, odom, scan, path):
        robot_map_x, robot_map_y = self.get_robot_pose_from_odom_in_map(odom)
        if robot_map_x is None:
            return None, None

        robot_linear_speed = float(odom.twist.twist.linear.x)
        robot_angular_speed = float(odom.twist.twist.angular.z)

        goal_map_x = float(path.poses[-1].pose.position.x)
        goal_map_y = float(path.poses[-1].pose.position.y)
        goal_distance_m = math.hypot(goal_map_x - robot_map_x, goal_map_y - robot_map_y)

        cleaned_ranges = []
        max_range = self.lidar_max_range_m

        for i in scan.ranges:
            if 0 < i <= max_range and not math.isnan(i) and math.isfinite(i):
                cleaned_ranges.append(i)
            else:
                cleaned_ranges.append(max_range)

        if cleaned_ranges:
            min_range_m = min(cleaned_ranges)
            normalized_lidar_sector = self.lidar_sector_min_distances(cleaned_ranges)
        else:
            min_range_m = max_range
            normalized_lidar_sector = [1.0] * self.lidar_sector

        cross_track_error_m = self.compute_cross_track_error(robot_map_x, robot_map_y, path)

        normalized_goal_distance = min(goal_distance_m / 20.0, 1.0)
        normalized_cross_track_error = min(cross_track_error_m / 2.0, 1.0)
        normalized_linear_velocity = max(min(robot_linear_speed / 1.0, 1.0), -1.0)
        normalized_angular_velocity = max(min(robot_angular_speed / 1.5, 1.0), -1.0)
        normalized_min_lidar_range = min_range_m / max_range

        state = [normalized_goal_distance,normalized_linear_velocity, normalized_angular_velocity, normalized_min_lidar_range, normalized_cross_track_error] + normalized_lidar_sector

        info = {"distance_goal": float(goal_distance_m),"min_range": float(min_range_m),"cross_track_error": float(cross_track_error_m)}

        return state, info

    
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

    
    def get_robot_pose_from_odom_in_map(self, odom):     
        odom_point = PointStamped()
        odom_point.header.frame_id = odom.header.frame_id
        #now = self.get_clock().now()
        #odom_point.header.stamp = self.get_clock().now().to_msg()
        odom_point.header.stamp = odom.header.stamp

        odom_point.point.x = odom.pose.pose.position.x
        odom_point.point.y = odom.pose.pose.position.y
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
    node = PPOProduct()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()