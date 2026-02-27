import math
import os

import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from geometry_msgs.msg import PointStamped
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


class PPOProduct(Node):
    def __init__(self):
        super().__init__("ppo_product")

        #Alap működés
        self.declare_parameter("control_hz", 10.0)
        self.control_hz = float(self.get_parameter("control_hz").value)

        self.declare_parameter("runs_dir", "./ppo_runs/grid_1")
        self.runs_dir = str(self.get_parameter("runs_dir").value)

        self.declare_parameter("model_file", "best_latest.pth")
        self.model_file = str(self.get_parameter("model_file").value)

        self.declare_parameter("reload_model", True)
        self.reload_model = bool(self.get_parameter("reload_model").value)

        self.declare_parameter("reload_check_period_s", 2.0)
        self.reload_check_period_s = float(self.get_parameter("reload_check_period_s").value)

        # Szenzor/state paramok
        self.declare_parameter("lidar_bins", 12)
        self.lidar_bins = int(self.get_parameter("lidar_bins").value)

        self.declare_parameter("lidar_max_range", 6.0)
        self.lidar_max_range_m = float(self.get_parameter("lidar_max_range").value)

        #Topicok
        self.declare_parameter("odom_topic", "/odom")
        self.odom_topic = str(self.get_parameter("odom_topic").value)

        self.declare_parameter("scan_topic", "/scan")
        self.scan_topic = str(self.get_parameter("scan_topic").value)

        self.declare_parameter("path_topic", "/planned_path_dilated")
        self.path_topic = str(self.get_parameter("path_topic").value)

        #MPPI / controller_server
        self.declare_parameter("controller_server_node", "/controller_server")
        self.controller_server_node = str(self.get_parameter("controller_server_node").value)

        #Action - MPPI param tartományok
        self.declare_parameter("vx_max_min", 0.20)
        self.declare_parameter("vx_max_max", 0.60)
        self.vx_max_min = float(self.get_parameter("vx_max_min").value)
        self.vx_max_max = float(self.get_parameter("vx_max_max").value)

        self.declare_parameter("cost_weight_min", 0.50)
        self.declare_parameter("cost_weight_max", 8.00)
        self.cost_weight_min = float(self.get_parameter("cost_weight_min").value)
        self.cost_weight_max = float(self.get_parameter("cost_weight_max").value)

        #Belső állapot
        self.best_model_path = os.path.join(self.runs_dir, self.model_file)
        self.last_model_mtime = 0.0

        self.latest_odom = None
        self.latest_scan = None
        self.latest_path = None

        self.is_configured = False
        self.last_vx_max = None
        self.last_cost_weight = None

        #PPO policy
        self.state_dim = 5 + self.lidar_bins
        self.action_dim = 2
        self.ppo = PPOTraining(state_dim=self.state_dim, action_dim=self.action_dim)

        # TF
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # controller_server set_parameters
        self.mppi_set_params_client = self.create_client(
            SetParameters,
            f"{self.controller_server_node}/set_parameters"
        )

        qos_path = QoSProfile(depth=10)
        qos_path.reliability = ReliabilityPolicy.RELIABLE
        qos_path.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self.sub_path = self.create_subscription(Path, self.path_topic, self._on_path, qos_path)

        qos_scan = QoSProfile(depth=10)
        qos_scan.reliability = ReliabilityPolicy.RELIABLE
        qos_scan.durability = DurabilityPolicy.VOLATILE

        self.sub_scan = self.create_subscription(LaserScan, self.scan_topic, self._on_scan, qos_scan)

        qos_odom = QoSProfile(depth=20)
        qos_odom.reliability = ReliabilityPolicy.RELIABLE
        qos_odom.durability = DurabilityPolicy.VOLATILE

        self.sub_odom = self.create_subscription(Odometry, self.odom_topic, self._on_odom, qos_odom)

        timer_period_s = 1.0 / max(1e-6, self.control_hz)
        self.timer = self.create_timer(timer_period_s, self._on_control_tick)

        if self.reload_model:
            self.reload_timer = self.create_timer(self.reload_check_period_s, self._check_and_reload_model)

        # első betöltés
        self._load_model_if_exists()

        self.get_logger().info("PPOProduct indul (EVAL ONLY).")
        self.get_logger().info(f"runs_dir: {self.runs_dir}")
        self.get_logger().info(f"model: {self.best_model_path}")
        self.get_logger().info(f"topics: odom={self.odom_topic} scan={self.scan_topic} path={self.path_topic}")
        self.get_logger().info(f"controller_server: {self.controller_server_node}")

    def _on_odom(self, msg):
        self.latest_odom = msg

    def _on_scan(self, msg):
        self.latest_scan = msg

    def _on_path(self, msg):
        self.latest_path = msg

    #Model load

    def _load_model_if_exists(self):
        """
        Betölti a runs_dir/best_latest.pth-t, ha létezik.
        Ha nincs, akkor logol és "default" MPPI paramot fog használni (nem állít semmit).
        """
        if not os.path.exists(self.best_model_path):
            self.get_logger().warn(f"Nincs modell: {self.best_model_path}")
            return False

        try:
            self.ppo.policy.save_file = self.best_model_path
            self.ppo.policy.load_from_file()
            self.ppo.copy_policy()

            self.last_model_mtime = os.path.getmtime(self.best_model_path)
            self.get_logger().info(f"Modell betöltve: {self.best_model_path}")
            return True

        except Exception as e:
            self.get_logger().error(f"Model betöltési hiba: {e}")
            return False

    def _check_and_reload_model(self):
        """
        Egyszerű hot-reload:
        ha a best_latest.pth módosult, újratölti és újrakonfigurálja az MPPI-t.
        """
        if not os.path.exists(self.best_model_path):
            return

        try:
            mtime = os.path.getmtime(self.best_model_path)
        except Exception:
            return

        if mtime <= self.last_model_mtime:
            return

        ok = self._load_model_if_exists()
        if ok:
            # új modell - új paramok beállítása a következő tickben
            self.is_configured = False
            self.get_logger().info("Új modell észlelve -> újrakonfigurálás fog történni.")

    #Main control
    def _on_control_tick(self):
        """
        Fő ciklus:
        - ha még nincs konfigurálva: state -> action_mean -> MPPI param set
        - utána csak logol ritkán
        """
        if self.latest_odom is None or self.latest_scan is None or self.latest_path is None:
            return

        if len(self.latest_path.poses) < 2:
            return

        if not self.is_configured:
            state, info = self._build_state_and_info(self.latest_odom, self.latest_scan, self.latest_path)
            if state is None:
                return

            vx_max, cost_weight = self._select_mppi_params_from_policy(state)

            if vx_max is None or cost_weight is None:
                # modell nincs / hiba -> nem állítunk semmit
                return

            self._set_mppi_parameters(vx_max=vx_max, cost_weight=cost_weight)

            self.last_vx_max = vx_max
            self.last_cost_weight = cost_weight
            self.is_configured = True

            self.get_logger().info(
                f"[CONFIG] MPPI vx_max={vx_max:.3f} cost_weight={cost_weight:.3f} "
                f"dist_goal={info['distance_goal']:.3f} min_r={info['min_range']:.2f} cte={info['cross_track_error']:.2f}"
            )

    # Policy - action - ranges
    def _select_mppi_params_from_policy(self, state_np):
        """
        Determinisztikus kiválasztás:
        - a policy eloszlás átlagát használjuk (mean), nem sample-t
        """
        if not os.path.exists(self.best_model_path):
            return None, None

        try:
            state_tensor = torch.tensor(state_np, dtype=torch.float32)

            with torch.no_grad():
                action_dist, _ = self.ppo.policy(state_tensor)
                action_tensor = action_dist.mean  # EVAL: mean

            action_np = action_tensor.squeeze(0).cpu().numpy().astype(np.float32)

            a0 = float(action_np[0])
            a1 = float(action_np[1])

            vx_max = self._map_action_to_range(a0, self.vx_max_min, self.vx_max_max)
            cost_weight = self._map_action_to_range(a1, self.cost_weight_min, self.cost_weight_max)
            return float(vx_max), float(cost_weight)

        except Exception as e:
            self.get_logger().error(f"Policy kiértékelési hiba: {e}")
            return None, None

    def _map_action_to_range(self, action_value, out_min, out_max):
        """
        [-1,1] -> [out_min,out_max]
        """
        a = float(max(-1.0, min(1.0, action_value)))
        t = (a + 1.0) * 0.5
        return float(out_min + t * (out_max - out_min))

    #MPPI param set
    def _make_double_param(self, name, value):
        param = RosParameter()
        param.name = name
        param.value = ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=float(value))
        return param

    def _set_mppi_parameters(self, vx_max, cost_weight):
        service_name = f"{self.controller_server_node}/set_parameters"

        if not self.mppi_set_params_client.wait_for_service(timeout_sec=0.5):
            self.get_logger().warn(f"Param service nem elérhető: {service_name}")
            return

        request = SetParameters.Request()
        request.parameters = [
            self._make_double_param("FollowPathMPPI.vx_max", vx_max),
            self._make_double_param("FollowPathMPPI.CostCritic.cost_weight", cost_weight),
        ]

        future = self.mppi_set_params_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=0.5)

        if future.result() is None:
            self.get_logger().warn("Param set: nincs válasz (timeout / hiba).")
            return

        for i, result in enumerate(future.result().results):
            if not result.successful:
                failed_name = request.parameters[i].name
                self.get_logger().error(f"Param set FAIL: {failed_name} reason={result.reason}")

    # State építés
    def _build_state_and_info(self, odom, scan, path):
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

        # Normalizálás: ugyanaz a logika, mint a Trainerben
        norm_goal_dist = min(distance_to_goal_m / 20.0, 1.0)
        norm_cte = min(cross_track_error_m / 2.0, 1.0)
        norm_v = float(np.clip(robot_v / 1.0, -1.0, 1.0))
        norm_w = float(np.clip(robot_w / 1.5, -1.0, 1.0))
        norm_min_r = float(np.clip(min_range_m / self.lidar_max_range_m, 0.0, 1.0))

        state = np.array(
            [norm_goal_dist, norm_v, norm_w, norm_min_r, norm_cte] + lidar_bins_norm,
            dtype=np.float32
        )

        info = {
            "distance_goal": float(distance_to_goal_m),
            "min_range": float(min_range_m),
            "cross_track_error": float(cross_track_error_m),
        }
        return state, info

    def _bin_lidar_min(self, scan_ranges):
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
        odom_point = PointStamped()
        odom_point.header.frame_id = odom.header.frame_id
        now = self.get_clock().now()
        odom_point.header.stamp = now.to_msg()

        odom_point.point.x = float(odom.pose.pose.position.x)
        odom_point.point.y = float(odom.pose.pose.position.y)
        odom_point.point.z = 0.0

        try:
            transform = self.tf_buffer.lookup_transform(
                "map",
                odom_point.header.frame_id,
                now,
                timeout=Duration(seconds=0.2),
            )
            map_point = do_transform_point(odom_point, transform)
            return float(map_point.point.x), float(map_point.point.y)

        except Exception as e:
            self.get_logger().error(f"TF hiba: {e}")
            return None, None


def main(args=None):
    rclpy.init(args=args)
    node = PPOProduct()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()