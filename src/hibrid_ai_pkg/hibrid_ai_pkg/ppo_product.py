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
    def __init__(self):
        super().__init__("ppo_product")

        self.declare_parameter("control_hz", 5.0)
        self.control_hz = float(self.get_parameter("control_hz").value)

        # ha true: csak egyszer állít MPPI paramokat (első jó state után), utána nem nyúl hozzá
        self.declare_parameter("set_once", True)
        self.set_once = bool(self.get_parameter("set_once").value)

        # ha set_once=False, ennyi lépésenként frissít (pl. 50)
        self.declare_parameter("update_every_steps", 50)
        self.update_every_steps = int(self.get_parameter("update_every_steps").value)

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

        self.declare_parameter("lidar_bins", 12)
        self.lidar_bins = int(self.get_parameter("lidar_bins").value)

        self.declare_parameter("lidar_max_range", 6.0)
        self.lidar_max_range_m = float(self.get_parameter("lidar_max_range").value)

        self.declare_parameter("runs_dir", "./ppo_runs")
        self.runs_dir = str(self.get_parameter("runs_dir").value)
        os.makedirs(self.runs_dir, exist_ok=True)

        self.declare_parameter("model_path", "")  # ha üres, akkor runs_dir/best_latest.pth
        model_path_param = str(self.get_parameter("model_path").value).strip()
        self.best_model_path = model_path_param if model_path_param else os.path.join(self.runs_dir, "best_latest.pth")

        self.latest_odom = None
        self.latest_scan = None
        self.latest_path = None
        self.latest_cmd_vel = None

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        #PPO policy
        self.state_dim = 5 + self.lidar_bins
        self.action_dim = 5
        self.ppo = PPOTraining(state_dim=self.state_dim, action_dim=self.action_dim)

        self.load_model_or_die()

        #service client
        self.mppi_set_params_client = self.create_client(SetParameters, f"{self.controller_server_node}/set_parameters")

        qos_path = QoSProfile(depth=10)
        qos_path.reliability = ReliabilityPolicy.RELIABLE
        qos_path.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.sub_path = self.create_subscription(Path, self.path_topic, self.path_callback, qos_path)

        qos_cmd = QoSProfile(depth=20)
        qos_cmd.reliability = ReliabilityPolicy.RELIABLE
        qos_cmd.durability = DurabilityPolicy.VOLATILE
        self.sub_cmd = self.create_subscription(Twist, self.cmd_vel_topic, self.cmd_callback, qos_cmd)

        qos_scan = QoSProfile(depth=10)
        qos_scan.reliability = ReliabilityPolicy.RELIABLE
        qos_scan.durability = DurabilityPolicy.VOLATILE
        self.sub_scan = self.create_subscription(LaserScan, self.scan_topic, self.scan_callback, qos_scan)

        qos_odom = QoSProfile(depth=20)
        qos_odom.reliability = ReliabilityPolicy.RELIABLE
        qos_odom.durability = DurabilityPolicy.VOLATILE
        self.sub_odom = self.create_subscription(Odometry, self.odom_topic, self.odom_callback, qos_odom)

        #ciklus
        self.step_index = 0
        self.did_set_params = False
        self.last_set_step = -10**9

        timer_period_sec = 1.0 / max(1e-6, self.control_hz)
        self.timer = self.create_timer(timer_period_sec, self.tick)

        self.get_logger().info("PPOProduct indul. mode=EVAL (no training)")
        self.get_logger().info(f"Model: {self.best_model_path}")
        self.get_logger().info(f"Nav2 controller_server: {self.controller_server_node}")
        self.get_logger().info(f"Topics: odom={self.odom_topic} scan={self.scan_topic} path={self.path_topic} cmd={self.cmd_vel_topic}")
        self.get_logger().info(f"set_once={self.set_once} update_every_steps={self.update_every_steps}")

    def odom_callback(self, msg):
        self.latest_odom = msg

    def scan_callback(self, msg):
        self.latest_scan = msg

    def path_callback(self, msg):
        self.latest_path = msg

    def cmd_callback(self, msg):
        self.latest_cmd_vel = msg


    # modell betöltése
    def load_model_or_die(self):
        if not os.path.exists(self.best_model_path):
            self.get_logger().error(f"Model nem található!!! : {self.best_model_path}")
            
        try:
            self.ppo.policy.save_file = self.best_model_path
            self.ppo.policy.load_from_file()
            self.ppo.copy_policy()
            self.get_logger().info("PPO policy betöltve...")
            return self.ppo
        
        except Exception as e:
            self.get_logger().error(f"Model betöltési hiba!!! : {e}")
            raise

    # vezérlési ciklua
    def tick(self):
        self.step_index = self.step_index + 1

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
            self.get_logger().warn(f"Path túl rövid: {len(self.latest_path.poses)}")
            return
        
        self.get_logger().debug("Minden input rendben mgjött!!!")


        if self.set_once and self.did_set_params:
            self.get_logger().debug("Paraméterek már beállítva, nem állítjuk újra...")
            return

        if (not self.set_once) and (self.step_index - self.last_set_step) < max(1, self.update_every_steps):
            self.get_logger().debug("Még nem kell a paraméter frissítés.")
            return

        state, info = self.build_state_and_info(self.latest_odom, self.latest_scan, self.latest_path)
        if state is None or info is None:
            self.get_logger().error("build_state_and_info() None-t adott vissza!!! State építés nem sikerült!!!")
            return

        action = self.select_action_mean(state)
        vx_max, wz_max, vx_std, wz_std, cost_weight = self.action_to_mppi(action)

        ok = self.set_mppi_parameters(vx_max=vx_max, wz_max=wz_max, vx_std=vx_std, wz_std=wz_std, cost_weight=cost_weight)
        
        if ok:
            self.did_set_params = True
            self.last_set_step = self.step_index
            self.get_logger().info(f"MPPI paramok beállítva: vx_max={vx_max:.3f} wz_max={wz_max:.3f} vx_std={vx_std:.3f} wz_std={wz_std:.3f} "
                                   f"CostCritic.cost_weight={cost_weight:.3f} | goal_dist={info['distance_goal']:.2f}m")


    #PPO-ból átlag akciót veszünk ki.
    def select_action_mean(self, state):
        state_tensor = torch.tensor(state, dtype=torch.float32)
        if state_tensor.ndim == 1:
            state_tensor = state_tensor.unsqueeze(0)

        with torch.no_grad():
            action_distribution, _ = self.ppo.policy(state_tensor)
            action_tensor = action_distribution.mean 

        action = action_tensor.squeeze(0).cpu().numpy().astype(np.float32)
        
        return action


    #PPo akciókat alakítjuk MPPI paraméterekké
    def action_to_mppi(self, action_array: np.ndarray):
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

        return float(vx_max), float(wz_max), float(vx_std), float(wz_std), float(cost_weight)


    # Action értéket [-1,1]-ből átmappel [out_min,out_max] tartományra...
    def map_action_to_range(self, action_value, out_min, out_max):
        safe_action = float(max(-1.0, min(1.0, action_value)))
        normalized = (safe_action + 1.0) * 0.5
        
        return float(out_min + normalized * (out_max - out_min))


    ##ROS2 double param üzenet létrehozása...
    def make_double_param(self, name, value):
        param = RosParameter()
        param.name = name
        param.value = ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=float(value))
        return param

    
    #MPPI paramok beállítása a controller_server set_parameters service-en...
    def set_mppi_parameters(self, vx_max, wz_max, vx_std, wz_std, cost_weight):
        service_name = f"{self.controller_server_node}/set_parameters"

        if not self.mppi_set_params_client.wait_for_service(timeout_sec=0.5):
            self.get_logger().error(f"Service nem elérhető: {service_name}")
            return False

        request = SetParameters.Request()
        request.parameters = [self.make_double_param("FollowPathMPPI.vx_max", vx_max),self.make_double_param("FollowPathMPPI.wz_max", wz_max),
                              self.make_double_param("FollowPathMPPI.vx_std", vx_std),self.make_double_param("FollowPathMPPI.wz_std", wz_std),
                              self.make_double_param("FollowPathMPPI.CostCritic.cost_weight", cost_weight)]

        future = self.mppi_set_params_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=0.8)

        if future.result() is None:
            self.get_logger().error("Timeout hiba a paraméterküldésnél...")
            return False

        ok = True
        for i, result in enumerate(future.result().results):
            if not result.successful:
                ok = False
                self.get_logger().error(f"Sikertelen: {request.parameters[i].name} reason={result.reason}")

        return ok

    #state building
    def build_state_and_info(self, odom, scan, path):
        robot_x, robot_y = self.get_robot_xy_in_map(odom)
        if robot_x is None:
            return None, None

        robot_v = float(odom.twist.twist.linear.x)
        robot_w = float(odom.twist.twist.angular.z)

        goal_x = float(path.poses[-1].pose.position.x)
        goal_y = float(path.poses[-1].pose.position.y)
        goal_distance_m = math.hypot(goal_x - robot_x, goal_y - robot_y)

        max_range = self.lidar_max_range_m
        
        clean_ranges = []
        for i in scan.ranges:
            if 0 < i <= max_range and (not math.isnan(i)) and math.isfinite(i):
                clean_ranges.append(i)
            else:
                clean_ranges.append(max_range)

        if clean_ranges:
            min_range_m = min(clean_ranges)
            normalized_lidar_bins = self.bin_lidar_min(clean_ranges)
        else:
            min_range_m = max_range
            normalized_lidar_bins = [1.0] * self.lidar_bins

        cross_track_error_m = self.compute_cross_track_error(robot_x, robot_y, path)

        # normalizálás
        normalized_goal_distance = min(goal_distance_m / 20.0, 1.0)
        normalized_cross_track_error = min(cross_track_error_m / 2.0, 1.0)
        normalized_linear_velocity = max(min(robot_v / 1.0, 1.0), -1.0)
        normalized_angular_velocity = max(min(robot_w / 1.5, 1.0), -1.0)
        normalized_min_lidar_range = min_range_m / max(1e-6, max_range)

        state = [normalized_goal_distance,normalized_linear_velocity, normalized_angular_velocity, normalized_min_lidar_range,normalized_cross_track_error,] + normalized_lidar_bins

        info = {"distance_goal": float(goal_distance_m), "min_range": float(min_range_m), "cross_track_error": float(cross_track_error_m)}
        
        return state, info
    

    #A lidar tartományt bin-ekre bontja és bin-enként minimumot ad vissza normalizálva...
    def bin_lidar_min(self, scan_ranges):
        total_points = len(scan_ranges)
        points_per_bin = max(1, total_points // self.lidar_bins)

        result = []
        for i in range(self.lidar_bins):
            start = i * points_per_bin
            end = min(total_points, (i + 1) * points_per_bin)

            if start >= total_points:
                minimum_distance = float(self.lidar_max_range_m)
            else:
                minimum_distance = float(np.min(scan_ranges[start:end]))

            result.append(minimum_distance / self.lidar_max_range_m)

        return result


    #Kiszámolja a robot legkisebbb távolságát a Path pontjaihoz...
    def compute_cross_track_error(self, robot_x, robot_y, path):
        if path is None or len(path.poses) == 0:
            return 0.0

        best_min_distance = float("inf")
        
        for pose_stamped in path.poses:
            path_x = float(pose_stamped.pose.position.x)
            path_y = float(pose_stamped.pose.position.y)
            euclides_ditance = math.hypot(path_x - robot_x, path_y - robot_y)
            
            if euclides_ditance < best_min_distance:
                best_min_distance = euclides_ditance

        return float(best_min_distance)

    #Odom pozíciót átalakítjamap frame-be TF segítségével...
    def get_robot_xy_in_map(self, odom):
        odom_point = PointStamped()
        odom_point.header.frame_id = odom.header.frame_id
        now = self.get_clock().now()
        odom_point.header.stamp = now.to_msg()

        odom_point.point.x = float(odom.pose.pose.position.x)
        odom_point.point.y = float(odom.pose.pose.position.y)
        odom_point.point.z = 0.0

        try:
            transform = self.tf_buffer.lookup_transform("map",odom_point.header.frame_id, now, timeout=Duration(seconds=0.2))
            map_point = do_transform_point(odom_point, transform)
            
            return float(map_point.point.x), float(map_point.point.y)

        except Exception as e:
            self.get_logger().warn(f"TF hiba van {e}")
            return None, None


def main(args=None):
    rclpy.init(args=args)
    node = PPOProduct()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()