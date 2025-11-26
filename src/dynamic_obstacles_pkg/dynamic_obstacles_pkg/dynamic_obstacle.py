import os
import math
import random
import subprocess

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from ament_index_python.packages import get_package_share_directory


class DynamicObstacleSpawner(Node):
    def __init__(self):
        super().__init__('dynamic_obstacle_spawner')

        self.min_delay = self.declare_parameter('min_delay', 12.0).value
        self.max_delay = self.declare_parameter('max_delay', 24.0).value
        self.world_name = self.declare_parameter('world_name', 'custom_world').value
        self.obstacle_name = self.declare_parameter('obstacle_name', 'dynamic_obstacle').value
        self.obstacle_z = self.declare_parameter('obstacle_z', 0.0).value

        # Modell elérési út (simulation_resources_pkg/worlds/dynamic_box.sdf)
        pkg_share = get_package_share_directory('simulation_resources_pkg')
        self.model_path = os.path.join(pkg_share, 'worlds', 'dynamic_box.sdf')
        
        qos_obstacle = QoSProfile(depth=10)
        qos_obstacle.reliability = ReliabilityPolicy.RELIABLE
        qos_obstacle.durability = DurabilityPolicy.VOLATILE

        # Random késleltetés
        self.delay = random.uniform(self.min_delay, self.max_delay)
        self.get_logger().info(
            f'Model path: {self.model_path}, delay={self.delay:.2f}s'
        )

        # Állapot
        self.current_path = None
        self.spawned = False
        self.start_time = self.get_clock().now()

        # Feliratkozás az ideális útvonalra
        self.create_subscription(Path, 'planned_path_dilated', self.path_callback, 10)
        
        self.obstacle_pub = self.create_publisher(PoseStamped, 'dynamic_obstacle', qos_obstacle)

        # Periodikus timer
        self.create_timer(0.5, self.timer_callback)

    # Callbacks
    def path_callback(self, msg: Path):
        if not msg.poses:
            return
        self.current_path = msg

    def timer_callback(self):
        if self.spawned:
            return

        elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        if elapsed < self.delay:
            return

        if self.current_path is None:
            return

        pose = self.select_pose_on_path_midpoint(self.current_path)
        if pose is None:
            self.get_logger().error('Nincs használható pont a path közepén.')
            return

        x = pose.pose.position.x
        y = pose.pose.position.y
        
        self.spawn_obstacle(x, y, self.obstacle_z)
        self.publish_dynamic_obstacle(x, y, self.obstacle_z)
        
        self.spawned = True

    # Path feldolgozás
    def select_pose_on_path_midpoint(self, path_msg: Path):
        poses = path_msg.poses
        if len(poses) < 2:
            return None

        # teljes hossz
        total_len = 0.0
        last = poses[0].pose.position
        for i in range(1, len(poses)):
            p = poses[i].pose.position
            dx = p.x - last.x
            dy = p.y - last.y
            total_len += math.hypot(dx, dy)
            last = p

        if total_len <= 1e-6:
            return poses[-1]

        half_dist = total_len / 2.0

        # félút körüli pont
        dist_acc = 0.0
        last = poses[0].pose.position
        for i in range(1, len(poses)):
            p = poses[i].pose.position
            dx = p.x - last.x
            dy = p.y - last.y
            seg = math.hypot(dx, dy)
            dist_acc += seg
            if dist_acc >= half_dist:
                return poses[i]
            last = p

        return poses[-1]

    # Spawn hívás
    def spawn_obstacle(self, x: float, y: float, z: float):
        if not os.path.exists(self.model_path):
            self.get_logger().error(f'Model file not found: {self.model_path}')
            return

        cmd = [
            'ros2', 'run', 'ros_gz_sim', 'create',
            '-world', self.world_name,
            '-file', self.model_path,
            '-name', self.obstacle_name,
            '-x', str(x),
            '-y', str(y),
            '-z', str(z),
        ]

        self.get_logger().info('Spawning dynamic obstacle...')
        self.get_logger().info(' '.join(cmd))

        result = subprocess.run(cmd, capture_output=True, text=True)
        if result.returncode == 0:
            self.get_logger().info('Dynamic obstacle spawned successfully.')
        else:
            self.get_logger().error(
                f'Spawn failed, code={result.returncode}\n'
                f'stdout:\n{result.stdout}\n'
                f'stderr:\n{result.stderr}'
            )
            
    def publish_dynamic_obstacle(self, x: float, y: float, z: float):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'   # a path is map frame-ben van

        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = z

        msg.pose.orientation.w = 1.0

        self.obstacle_pub.publish(msg)
        self.get_logger().info(f'Dynamic obstacle published on /dynamic_obstacle at (x={x:.2f}, y={y:.2f})')


def main(args=None):
    rclpy.init(args=args)
    node = DynamicObstacleSpawner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
