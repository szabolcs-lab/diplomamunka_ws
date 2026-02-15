import math
import os
import numpy as np
import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray

import torch

from .actor_critic_network import ActorCriticNetwork, action_to_shaping


class PPOProduct(Node):
    def __init__(self):
        super().__init__("ppo_product")

        self.get_logger().info("PPO Product node inicializálás...")

        # paraméterek
        self.declare_parameter("model_path", "./ppo_runs/best_latest.pth")
        self.declare_parameter("control_hz", 2.0)

        self.declare_parameter("lidar_bins", 12)
        self.declare_parameter("lidar_max_range", 6.0)

        self.declare_parameter("max_offset_m", 0.10)
        self.declare_parameter("offset_limit", 0.10)
        self.declare_parameter("smooth_max", 0.25)

        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("path_topic", "/planned_path_smoother")
        self.declare_parameter("params_topic", "/smoother_params")

        # beolvasás
        self.model_path = str(self.get_parameter("model_path").value)
        self.control_hz = float(self.get_parameter("control_hz").value)

        self.lidar_bins = int(self.get_parameter("lidar_bins").value)
        self.lidar_max_range = float(self.get_parameter("lidar_max_range").value)

        self.max_offset_m = float(self.get_parameter("max_offset_m").value)
        self.offset_limit = float(self.get_parameter("offset_limit").value)
        self.smooth_max = float(self.get_parameter("smooth_max").value)

        self.odom_topic = str(self.get_parameter("odom_topic").value)
        self.scan_topic = str(self.get_parameter("scan_topic").value)
        self.path_topic = str(self.get_parameter("path_topic").value)
        self.params_topic = str(self.get_parameter("params_topic").value)

        # state dim = [distance_goal, v, w, min_range] + lidar_bins
        self.state_dim = 4 + self.lidar_bins
        self.policy = ActorCriticNetwork(n_inputs=self.state_dim, n_actions=2)

        self.load_model()

        # cache
        self.last_odom = None
        self.last_scan = None
        self.last_path = None

        # ROS IO
        self.sub_odom = self.create_subscription(Odometry, self.odom_topic, self.odom_callback, 10)
        self.sub_scan = self.create_subscription(LaserScan, self.scan_topic, self.scan_callback, 10)
        self.sub_path = self.create_subscription(Path, self.path_topic, self.path_callback, 10)

        self.pub_params = self.create_publisher(Float32MultiArray, self.params_topic, 10)

        period = 1.0 / max(1e-6, self.control_hz)
        self.timer = self.create_timer(period, self.on_timer)

        self.get_logger().info("PPO Product node elindult...")


    def load_model(self):
        if not os.path.exists(self.model_path):
            self.get_logger().error(f"Model nem található: {self.model_path}")
            return

        try:
            self.policy.save_file = self.model_path
            self.policy.load_from_file()
            self.policy.eval()
            self.get_logger().warning(f"Model betöltve: {self.model_path}")
        except Exception as e:
            self.get_logger().error(f"Model betöltési hiba: {e}")


    def odom_callback(self, msg: Odometry):
        self.last_odom = msg

    def scan_callback(self, msg: LaserScan):
        self.last_scan = msg

    def path_callback(self, msg: Path):
        self.last_path = msg


    def on_timer(self):
        if self.last_odom is None or self.last_scan is None or self.last_path is None:
            return
        if len(self.last_path.poses) < 2:
            return

        state = self.build_state(self.last_odom, self.last_scan, self.last_path)
        state_tensor = torch.tensor(state, dtype=torch.float32)

        with torch.no_grad():
            action_distribution, _ = self.policy(state_tensor)
            action = action_distribution.mean  # determinisztikus (nem sample)

        action_np = action.squeeze(0).cpu().numpy().astype(np.float32)

        offset, smooth = action_to_shaping(torch.tensor(action_np), max_offset_m=self.max_offset_m)

        # limit
        offset = float(max(-self.offset_limit, min(self.offset_limit, float(offset))))
        smooth = float(max(0.0, min(self.smooth_max, float(smooth))))

        out = Float32MultiArray()
        out.data = [offset, smooth]
        self.pub_params.publish(out)


    def build_state(self, odom: Odometry, scan: LaserScan, path: Path):
        robot_x = float(odom.pose.pose.position.x)
        robot_y = float(odom.pose.pose.position.y)
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
            for bin_index in range(self.lidar_bins):
                start_index = bin_index * points_per_bin
                end_index = min(total_lidar_points, (bin_index + 1) * points_per_bin)

                if start_index < total_lidar_points:
                    min_distance_in_bin = float(np.min(ranges[start_index:end_index]))
                else:
                    min_distance_in_bin = float(self.lidar_max_range)

                lidar_vector.append(min_distance_in_bin / self.lidar_max_range)

        return np.array(
            [distance_goal, robot_speed, robot_turn_speed, min_range] + lidar_vector,
            dtype=np.float32
        )


def main(args=None):
    rclpy.init(args=args)
    node = PPOProduct()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
