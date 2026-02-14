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
        self.model_path = self.get_parameter("model_path").value
        self.control_hz = self.get_parameter("control_hz").value
        self.lidar_bins = self.get_parameter("lidar_bins").value
        self.lidar_max_range = self.get_parameter("lidar_max_range").value
        self.max_offset_m = self.get_parameter("max_offset_m").value
        self.offset_limit = self.get_parameter("offset_limit").value
        self.smooth_max = self.get_parameter("smooth_max").value

        self.odom_topic = self.get_parameter("odom_topic").value
        self.scan_topic = self.get_parameter("scan_topic").value
        self.path_topic = self.get_parameter("path_topic").value
        self.params_topic = self.get_parameter("params_topic").value

        #state dim
        self.state_dim = 4 + self.lidar_bins
        self.policy = ActorCriticNetwork(self.state_dim, 2)

        self.load_model()

        self.last_odom = None
        self.last_scan = None
        self.last_path = None

        self.sub_odom = self.create_subscription(Odometry, self.odom_topic, self.odom_callback, 10)
        self.sub_scan = self.create_subscription(LaserScan, self.scan_topic, self.scan_callback, 10)
        self.sub_path = self.create_subscription(Path, self.path_topic, self.path_callback, 10)

        self.pub_params = self.create_publisher(Float32MultiArray, self.params_topic, 10)

        period = 1.0 / self.control_hz
        self.timer = self.create_timer(period, self.on_timer)

        self.get_logger().info("PPO Product node elindult...")


    def load_model(self):
        if not os.path.exists(self.model_path):
            self.get_logger().error(f"Model nem található: {self.model_path}")
            return

        checkpoint = torch.load(self.model_path, map_location="cpu")
        self.policy.load_state_dict(checkpoint["model_state_dict"])
        self.policy.eval()

        self.get_logger().info(f"Model betöltve: {self.model_path}")


    def odom_callback(self, msg):
        self.last_odom = msg

    def scan_callback(self, msg):
        self.last_scan = msg

    def path_callback(self, msg):
        self.last_path = msg


    def on_timer(self):
        if self.last_odom is None or self.last_scan is None or self.last_path is None:
            return
        if len(self.last_path.poses) < 2:
            return

        state = self.build_state(self.last_odom, self.last_scan, self.last_path)

        state_tensor = torch.tensor(state, dtype=torch.float32)

        with torch.no_grad():
            distribution, _ = self.policy(state_tensor)
            action = distribution.mean  # determinisztikus

        action_np = action.squeeze(0).numpy()
        offset, smooth = action_to_shaping(torch.tensor(action_np), max_offset_m=self.max_offset_m)

        # limit
        offset = float(max(-self.offset_limit, min(self.offset_limit, offset)))
        smooth = float(max(0.0, min(self.smooth_max, smooth)))

        msg = Float32MultiArray()
        msg.data = [offset, smooth]
        self.pub_params.publish(msg)


    def build_state(self, odom, scan, path):
        robot_x = odom.pose.pose.position.x
        robot_y = odom.pose.pose.position.y
        robot_speed = odom.twist.twist.linear.x
        robot_turn_speed = odom.twist.twist.angular.z

        goal_x = path.poses[-1].pose.position.x
        goal_y = path.poses[-1].pose.position.y
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
                    min_distance_in_bin = float(np.min(ranges[start_index:end_index]))
                else:
                    min_distance_in_bin = self.lidar_max_range

                lidar_vector.append(min_distance_in_bin / self.lidar_max_range)

        return np.array([distance_goal, robot_speed, robot_turn_speed, min_range] + lidar_vector, dtype=np.float32)



def main(args=None):
    rclpy.init(args=args)
    node = PPOProduct()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
