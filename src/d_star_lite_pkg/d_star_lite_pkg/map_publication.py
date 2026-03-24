import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from nav_msgs.msg import OccupancyGrid, MapMetaData
from std_msgs.msg import Header
import numpy as np

class MapPublication(Node):
    """
    A node egy OccupancyGridet fog publikálni a 'map' topicon, amit egy csv-ből olvas be
    """
    
    def __init__(self):
        super().__init__('map_publication')
        
        self.get_logger().info('Map publication node indul....')
        
        self.declare_parameter('map_file', '')
        map_file = self.get_parameter('map_file').get_parameter_value().string_value
        
        self.grid = np.loadtxt(map_file, delimiter=',').astype(np.int8)
        
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
        
        data = []
        
        for y in range(self.grid.shape[0]):
            for x in range(self.grid.shape[1]):
                cell = self.grid[y, x]
                
                if cell ==1:
                    data.append(100)
                else:
                    data.append(0)
                    
        self.map_msg.data = data
        
        self.timer = self.create_timer(1.0, self.publish_map)
        
        self.get_logger().info('A map_publication node inicializálva....')
        
    def publish_map(self):
        
        self.map_msg.header.stamp = self.get_clock().now().to_msg()
        self.map_publisher.publish(self.map_msg)
        self.get_logger().info('Az Occupancy grid publikálva....')
        
def main(args=None):
    rclpy.init(args=args)
    node = MapPublication()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
        
if __name__ == '__main__':
    main()
