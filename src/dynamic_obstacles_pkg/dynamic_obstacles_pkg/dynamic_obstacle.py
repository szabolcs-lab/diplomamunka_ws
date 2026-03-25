import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from ament_index_python.packages import get_package_share_directory
import os
import math
import random
import subprocess # külső parancs futtatásához kell


class DynamicObstacleSpawner(Node):
    '''
        A node vár egy random ideig, közben kap egy útvonalat (planned_path_dilated), majd a kapott út felénél kiválaszt egy pontot,
        ott az útra ráhelyez egy akadályt Gazebo-ban és publikálja a /dynamic_obstacle topicra, 
        hogy más node-ok is tudjanak róla.
    '''
    def __init__(self):
        super().__init__('dynamic_obstacle_spawner')

        self.min_delay = self.declare_parameter('min_delay', 17.0).value
        self.max_delay = self.declare_parameter('max_delay', 20.0).value
        self.world_name = self.declare_parameter('world_name', 'custom_world').value
        self.obstacle_name = self.declare_parameter('obstacle_name', 'dynamic_obstacle').value
        self.obstacle_z = self.declare_parameter('obstacle_z', 0.0).value

        # simulation_resources_pkg/worlds/dynamic_box.sdf
        pkg_share = get_package_share_directory('simulation_resources_pkg')
        self.model_path = os.path.join(pkg_share, 'worlds', 'dynamic_box.sdf')
        
        # Random késleltetés min_delay = 17 és max_ delay = 24 között
        self.delay = random.uniform(self.min_delay, self.max_delay)
        self.current_path = None 
        self.spawned = False 
        self.start_time = self.get_clock().now()
        
        qos_obstacle = QoSProfile(depth=10)
        qos_obstacle.reliability = ReliabilityPolicy.RELIABLE 
        qos_obstacle.durability = DurabilityPolicy.TRANSIENT_LOCAL
        
        self.obstacle_publisher = self.create_publisher(PoseStamped, 'dynamic_obstacle', qos_obstacle)

        self.create_subscription(Path, '/planned_path_dilated', self.path_callback, 10)
        
        self.create_timer(0.5, self.timer_callback)

    def path_callback(self, msg):
        if not msg.poses:
            return
        self.current_path = msg

    def timer_callback(self):
        if self.spawned:
            return

        # idő ellenőrzés, ssak akkor engedi tovább a kódot, ha letelt a random késleltetés
        time_checker = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        if time_checker < self.delay:
            return

        if self.current_path is None:
            return

        # kiválasztjuk az útvonal közepét a függvény meghívásával
        pose = self.middle_point_on_the_path(self.current_path)
        
        if pose is None:
            self.get_logger().error('Nincs használható pont a path közepén...')
            return

        half_x_point = pose.pose.position.x
        half_y_point = pose.pose.position.y
        
        self.spawn_obstacle(half_x_point, half_y_point, self.obstacle_z)
        self.publish_dynamic_obstacle(half_x_point, half_y_point, self.obstacle_z)
        
        self.spawned = True

    # A függvény a path teljes hosszát kiszámolja, majd a kb. a felénél lévő pozíciót visszaadja
    def middle_point_on_the_path(self, path_msg):
        poses = path_msg.poses
        
        if len(poses) < 2:
            return None

        total_length = 0.0
        last = poses[0].pose.position
        
        for i in range(1, len(poses)):
            current_point= poses[i].pose.position
            delta_x = current_point.x - last.x
            delta_y = current_point.y - last.y
            total_length = total_length + math.hypot(delta_x, delta_y)
            last = current_point

        if total_length <= 1e-6:
            return poses[-1]

        half_length = total_length / 2.0

        sum_distance = 0.0
        last = poses[0].pose.position
        
        for i in range(1, len(poses)):
            current_point = poses[i].pose.position
            delta_x = current_point.x - last.x
            delta_y = current_point.y - last.y     
            sum_distance = sum_distance +math.hypot(delta_x, delta_y)
            
            if sum_distance >= half_length:
                return poses[i]
            
            last = current_point
            
        return poses[-1]

    # itt megy a spawn-olás
    def spawn_obstacle(self, x, y, z):
        if not os.path.exists(self.model_path):
            self.get_logger().error(f'Az sdf fájl ami az akadály modeljét tartalmazza nem található: {self.model_path}')
            return

        # ros_gz_sim objektum-spawn parancs
        # mint cmd-ben : 
        # ros2 run ros_gz_sim create -world 'self.world_name' -file 'sself.model_path' -name 'self.obstacle_name' -x str(x) -y str(y) -z str(z)
        cmd = ['ros2', 'run', 'ros_gz_sim', 'create', '-world', self.world_name, '-file', self.model_path, '-name', self.obstacle_name,
               '-x', str(x), '-y', str(y), '-z', str(z)]

        self.get_logger().info('Akadály spawn-olása...')
        
        # összefűzöm a cmd lista elemeit 
        # # ros2 run ros_gz_sim create -world 'self.world_name' -file 'sself.model_path' -name 'self.obstacle_name' -x str(x) -y str(y) -z str(z)
        self.get_logger().info(' '.join(cmd))

        # ROS2 spawn parancs futtatása, mintha a terminálba írtuk volna be
        result = subprocess.run(cmd, capture_output=True, text=True)
        if result.returncode == 0:
            self.get_logger().info('Az akadály spawn-olása suikerült...')
        else:
            self.get_logger().error(f'Spawn-olás nem sikerült...')
    
    
    # létrehozok egy PoseStamped üzenetet, amit publikálok, majd a dynamic_obstacle topicra       
    def publish_dynamic_obstacle(self, x, y, z):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'

        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = z
        msg.pose.orientation.w = 1.0

        self.obstacle_publisher.publish(msg)
        self.get_logger().info(f'Az akadály publikálva a /dynamic_obstacle topcin, a következő pontokra (x={x:.2f}, y={y:.2f})')


def main(args=None):
    rclpy.init(args=args)
    node = DynamicObstacleSpawner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
