import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Header
from nav_msgs.msg import MapMetaData
import os
import numpy as np
from ament_index_python.packages import get_package_share_directory

class MapPublication(Node):
    def __init__(self):
        super().__init__('map_publication')
        
        self.get_logger().info('map_publication node is initialization....')
        
        self.declare_parameter('map_file', 'occupancy_grid_1.csv')
        map_file = self.get_parameter('map_file').get_parameter_value().string_value
        
        maps_dir = os.path.join(get_package_share_directory('rrt_star_pkg'), 'maps')
        csv_path = os.path.join(maps_dir, map_file)
        
        self.get_logger().info(f"Loading occupancy grid from: {csv_path}")
        self.grid = np.loadtxt(csv_path, delimiter=',').astype(np.int8)
        
        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        
        self.map_publisher = self.create_publisher(OccupancyGrid, 'map', qos)
        
        self.map_msg = OccupancyGrid()
        self.map_msg.header = Header()
        self.map_msg.header.frame_id = 'map'
        
        self.map_msg.info = MapMetaData()
        self.map_msg.info.height = self.grid.shape[0]
        self.map_msg.info.width = self.grid.shape[1]
        self.map_msg.info.resolution = 0.1
        
        self.map_msg.info.origin.position.x = - self.map_msg.info.width * self.map_msg.info.resolution / 2.0
        self.map_msg.info.origin.position.y = - self.map_msg.info.height * self.map_msg.info.resolution / 2.0
        self.map_msg.info.origin.position.z = 0.0
        self.map_msg.info.origin.orientation.w = 1.0
        
        self.map_msg.data = (self.grid.flatten() * 100).tolist()
        
        self.timer = self.create_timer(1.0, self.publish_map)
        
        self.get_logger().info('map_publication node has been initialized....')
        
    def publish_map(self):
        self.map_msg.header.stamp = self.get_clock().now().to_msg()
        self.map_publisher.publish(self.map_msg)
        self.get_logger().info('Published map message...')
        
def main(args=None):
    rclpy.init(args=args)
    node = MapPublication()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()
