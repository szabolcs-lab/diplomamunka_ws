import os
import math
import numpy as np

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import LaserScan

from geometry_msgs.msg import PointStamped
from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_point
from rclpy.duration import Duration

from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter as RosParameter
from rcl_interfaces.msg import ParameterValue
from rcl_interfaces.msg import ParameterType

import torch

from .actor_critic_network import ActorCriticNetwork


class PPOProduct(Node):
    """
    PPOProduct (production):
    - betölti a best_latest.pth modellt
    - kiszámolja a state-et ugyanúgy, mint a trainer
    - egyszer (1x) beállítja az MPPI paramétereket a controller_server-en
    """

    def __init__(self):
        super().__init__("ppo_product")

        # -------- Paraméterek (ugyanazok a lényegesek, mint trainerben) --------
        self.declare_parameter("model_path", "./ppo_runs/grid_1/best_latest.pth")
        self.declare_parameter("lidar_bins", 12)
        self.declare_parameter("lidar_max_range", 6.0)

        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("path_topic", "/planned_path_dilated")

        self.declare_parameter("controller_server_node", "/controller_server")

        # MPPI param tartományok (UGYANAZ, mint trainerben!)
        self.declare_parameter("vx_max_min", 0.20)
        self.declare_parameter("vx_max_max", 0.60)
        self.declare_parameter("cost_weight_min", 0.50)
        self.declare_parameter("cost_weight_max", 8.00)

        # -------- Beolvasás --------
        self.model_path = str(self.get_parameter("model_path").value)

        self.lidar_bins = int(self.get_parameter("lidar_bins").value)
        self.lidar_max_range = float(self.get_parameter("lidar_max_range").value)

        self.odom_topic = str(self.get_parameter("odom_topic").value)
        self.scan_topic = str(self.get_parameter("scan_topic").value)
        self.path_topic = str(self.get_parameter("path_topic").value)

        self.controller_server_node = str(self.get_parameter("controller_server_node").value)

        self.vx_max_min = float(self.get_parameter("vx_max_min").value)
        self.vx_max_max = float(self.get_parameter("vx_max_max").value)
        self.cost_weight_min = float(self.get_parameter("cost_weight_min").value)
        self.cost_weight_max = float(self.get_parameter("cost_weight_max").value)

        # -------- TF (ugyanaz a logika, mint trainerben) --------
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # -------- PPO policy (ugyanaz a state dim, mint trainerben) --------
        self.state_dim = 5 + self.lidar_bins
        self.action_dim = 2
        self.policy = ActorCriticNetwork(n_inputs=self.state_dim, n_actions=self.action_dim)

        self._load_model()

        # -------- Cache üzenetek --------
        self.last_odom = None
        self.last_scan = None
        self.last_path = None

        # -------- MPPI param service kliens --------
        self.set_params_client = self.create_client(
            SetParameters, f"{self.controller_server_node}/set_parameters"
        )

        # FONTOS: csak egyszer állítunk paramot
        self.mppi_params_sent = False

        # -------- ROS feliratkozások --------
        self.sub_odom = self.create_subscription(Odometry, self.odom_topic, self._cb_odom, 20)
        self.sub_scan = self.create_subscription(LaserScan, self.scan_topic, self._cb_scan, 10)
        self.sub_path = self.create_subscription(Path, self.path_topic, self._cb_path, 10)

        # Timer csak arra kell, hogy megvárjuk az első odom/scan/path üzeneteket
        self.timer = self.create_timer(0.2, self._try_set_mppi_once)

        self.get_logger().info("PPOProduct elindult.")
        self.get_logger().info(f"Model: {self.model_path}")
        self.get_logger().info(f"MPPI node: {self.controller_server_node}")
        self.get_logger().info("MPPI param set: egyszer induláskor (nem spammel).")

    # ---------------- Callbacks ----------------
    def _cb_odom(self, msg: Odometry):
        self.last_odom = msg

    def _cb_scan(self, msg: LaserScan):
        self.last_scan = msg

    def _cb_path(self, msg: Path):
        self.last_path = msg

    # ---------------- Model load ----------------
    def _load_model(self):
        if not os.path.exists(self.model_path):
            self.get_logger().error(f"Model nem található: {self.model_path}")
            return

        try:
            self.policy.save_file = self.model_path
            self.policy.load_from_file()
            self.policy.eval()
            self.get_logger().info("Model betöltve (best_latest.pth).")
        except Exception as e:
            self.get_logger().error(f"Model betöltési hiba: {e}")

    # ---------------- Main: set MPPI once ----------------
    def _try_set_mppi_once(self):
        # már beállítottuk -> kész, timer leállítható
        if self.mppi_params_sent:
            return

        # várjuk meg az első adatok érkezését, különben nincs state
        if self.last_odom is None or self.last_scan is None or self.last_path is None:
            return
        if len(self.last_path.poses) < 2:
            return

        state, _info = self._build_state_like_trainer(self.last_odom, self.last_scan, self.last_path)
        if state is None:
            return

        # 1) policy -> action (deterministic)
        st = torch.tensor(state, dtype=torch.float32)
        with torch.no_grad():
            action_dist, _ = self.policy(st)
            action = action_dist.mean

        action_np = action.squeeze(0).cpu().numpy().astype(np.float32)

        # 2) action [-1,1] -> param tartomány
        vx_max = self._map_action_to_range(float(action_np[0]), self.vx_max_min, self.vx_max_max)
        cost_w = self._map_action_to_range(float(action_np[1]), self.cost_weight_min, self.cost_weight_max)

        # 3) service megvárása normálisan (nem 0.5s!)
        if not self.set_params_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn("controller_server /set_parameters nem elérhető (5s timeout). Próbálom később...")
            return

        ok = self._set_mppi_params_blocking(vx_max=vx_max, cost_weight=cost_w)
        if ok:
            self.mppi_params_sent = True
            self.get_logger().info(f"[MPPI SET ONCE] vx_max={vx_max:.3f} cost_weight={cost_w:.3f}")

    def _map_action_to_range(self, a: float, out_min: float, out_max: float) -> float:
        # a in [-1,1] -> [out_min, out_max]
        a = float(max(-1.0, min(1.0, a)))
        t = (a + 1.0) * 0.5
        return float(out_min + t * (out_max - out_min))

    # ---------------- Param set (blocking, egyszer) ----------------
    def _make_double_param(self, name: str, value: float) -> RosParameter:
        p = RosParameter()
        p.name = name
        p.value = ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=float(value))
        return p

    def _set_mppi_params_blocking(self, vx_max: float, cost_weight: float) -> bool:
        req = SetParameters.Request()
        req.parameters = [
            self._make_double_param("FollowPathMPPI.vx_max", vx_max),
            self._make_double_param("FollowPathMPPI.CostCritic.cost_weight", cost_weight),
        ]

        future = self.set_params_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)

        if future.result() is None:
            self.get_logger().warn("Param set: nincs válasz (timeout / hiba).")
            return False

        results = future.result().results
        for i, res in enumerate(results):
            if not res.successful:
                pname = req.parameters[i].name
                self.get_logger().warn(f"Param set FAIL: {pname} reason={res.reason}")
                return False

        return True

    # ---------------- State: ugyanaz, mint trainer ----------------
    def _build_state_like_trainer(self, odom: Odometry, scan: LaserScan, path: Path):
        # Robot pozíció map frame-ben TF-fel (ugyanaz, mint trainer)
        robot_x, robot_y = self._robot_xy_in_map(odom)
        if robot_x is None:
            return None, None

        # sebességek odomból
        robot_linear_speed = float(odom.twist.twist.linear.x)
        robot_angular_speed = float(odom.twist.twist.angular.z)

        # cél a path utolsó pontja (map frame)
        goal_x = float(path.poses[-1].pose.position.x)
        goal_y = float(path.poses[-1].pose.position.y)
        distance_to_goal_m = math.hypot(goal_x - robot_x, goal_y - robot_y)

        # lidar feldolgozás, bin-ek
        ranges = np.array(scan.ranges, dtype=np.float32)
        ranges = np.where(np.isfinite(ranges), ranges, self.lidar_max_range)
        ranges = np.clip(ranges, 0.0, self.lidar_max_range)

        if len(ranges) == 0:
            lidar_bins_norm = [1.0] * self.lidar_bins
            min_range_m = float(self.lidar_max_range)
        else:
            min_range_m = float(np.min(ranges))
            total = len(ranges)
            points_per_bin = max(1, total // self.lidar_bins)

            lidar_bins_norm = []
            for i in range(self.lidar_bins):
                start = i * points_per_bin
                end = min(total, (i + 1) * points_per_bin)
                if start < total:
                    min_in_bin = float(np.min(ranges[start:end]))
                else:
                    min_in_bin = float(self.lidar_max_range)
                lidar_bins_norm.append(min_in_bin / self.lidar_max_range)

        cross_track_error_m = self._calc_cross_track_error(robot_x, robot_y, path)

        # Normalizálások (Ugyanaz, mint trainerben!)
        # 20m felett 1.0 (mert nálad 20x20 map -> stabil skála)
        norm_goal_dist = min(distance_to_goal_m / 20.0, 1.0)
        # 2m felett 1.0 (mert a pályától 2m-nél már nagyon rossz)
        norm_cte = min(cross_track_error_m / 2.0, 1.0)
        # 1.0 m/s körül “max”
        norm_v = np.clip(robot_linear_speed / 1.0, -1.0, 1.0)
        # 1.5 rad/s körül “max”
        norm_w = np.clip(robot_angular_speed / 1.5, -1.0, 1.0)
        # lidar max-hoz normálva
        norm_min_r = np.clip(min_range_m / self.lidar_max_range, 0.0, 1.0)

        state = np.array(
            [norm_goal_dist, norm_v, norm_w, norm_min_r, norm_cte] + lidar_bins_norm,
            dtype=np.float32,
        )

        info = {
            "distance_goal": float(distance_to_goal_m),
            "min_range": float(min_range_m),
            "cross_track_error": float(cross_track_error_m),
        }
        return state, info

    def _calc_cross_track_error(self, robot_x: float, robot_y: float, path: Path) -> float:
        best = 1e9
        for pose_stamped in path.poses:
            px = float(pose_stamped.pose.position.x)
            py = float(pose_stamped.pose.position.y)
            d = math.hypot(px - robot_x, py - robot_y)
            if d < best:
                best = d
        return float(best)

    def _robot_xy_in_map(self, odom: Odometry):
        odom_point = PointStamped()
        odom_point.header.frame_id = odom.header.frame_id
        now = self.get_clock().now()
        odom_point.header.stamp = now.to_msg()

        odom_point.point.x = float(odom.pose.pose.position.x)
        odom_point.point.y = float(odom.pose.pose.position.y)
        odom_point.point.z = 0.0

        try:
            tf = self.tf_buffer.lookup_transform(
                "map",
                odom_point.header.frame_id,
                now,
                timeout=Duration(seconds=0.2),
            )
            map_point = do_transform_point(odom_point, tf)
            return float(map_point.point.x), float(map_point.point.y)
        except Exception as e:
            self.get_logger().warn(f"TF hiba: {e}")
            return None, None


def main(args=None):
    rclpy.init(args=args)
    node = PPOProduct()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()