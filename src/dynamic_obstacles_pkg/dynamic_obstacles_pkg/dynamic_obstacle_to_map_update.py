import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
import math
import os
import numpy as np


class DynamicObstacleToMapUpdate(Node):
    def __init__(self):
        super().__init__('dynamic_obstacle_to_map_update')

        self.static_map_topic = self.declare_parameter('static_map_topic', 'map').value
        self.dynamic_map_topic = self.declare_parameter('dynamic_map_topic', 'map_dynamic').value

        # fél hosszak (méterben) X és Y irányban
        self.obstacle_half_extent_x = self.declare_parameter('obstacle_half_extent_x', 0.25).value
        self.obstacle_half_extent_y = self.declare_parameter('obstacle_half_extent_y', 1.25).value

        self.get_logger().info(f'Statikus map topicja: {self.static_map_topic}, dinamikus map topicja: {self.dynamic_map_topic}')

        # ide mentem az utoljára kapott statikus OccupancyGridet
        self.static_map_msg = None 
        
        # itt fogom tráolni az akadályokat
        self.obstacle_list = []          

        qos_map = QoSProfile(depth=1)
        qos_map.reliability = ReliabilityPolicy.RELIABLE
        qos_map.durability = DurabilityPolicy.TRANSIENT_LOCAL
        
        self.create_subscription(OccupancyGrid, self.static_map_topic, self.map_callback, qos_map)
        
        qos_obstacle = QoSProfile(depth=10)
        qos_obstacle.reliability = ReliabilityPolicy.RELIABLE
        qos_obstacle.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self.create_subscription(PoseStamped, 'dynamic_obstacle', self.obstacle_callback, qos_obstacle)

        self.map_publisher = self.create_publisher(OccupancyGrid, self.dynamic_map_topic, qos_map)

    def map_callback(self, msg):
        self.static_map_msg = msg
        self.get_logger().info(f'Statikus map: {msg.info.height}x{msg.info.width}, resolution={msg.info.resolution:.3f}')

        # ha már van akadály, akkor generálok egy dinamikus mapet
        self.publish_dynamic_map()

    def obstacle_callback(self, msg):
        
        if msg.header.frame_id not in ['', 'map']:
            self.get_logger().warn(f'Nincs map {msg.header.frame_id}......')

        map_x = msg.pose.position.x
        map_y = msg.pose.position.y

        self.obstacle_list.append((map_x, map_y))
        self.get_logger().info(f'A dinamikus akadály a világban az x={map_x:.2f}, y={map_y:.2f} koordinátán van......')

        #az akadály érkezése után új map megy
        self.publish_dynamic_map()

    # Dinamikus map generálása
    def publish_dynamic_map(self):
        if self.static_map_msg is None:
            self.get_logger().warn('Nincs még statikus map, így nem tudok dinamikus map-et publikálni......')
            return

        # statikus map megy numpy array-be, 1D lista
        base_grid_map_numpy = np.array(self.static_map_msg.data, dtype=np.int16)
        height = self.static_map_msg.info.height
        width = self.static_map_msg.info.width
        base_grid_map_numpy = base_grid_map_numpy.reshape((height, width))

        dynamic_grid_map = base_grid_map_numpy.copy()

        # paraméterek ahhoz, hogy a világot át tudjam konvertálni gridre
        resolution = float(self.static_map_msg.info.resolution)
        origin_x = float(self.static_map_msg.info.origin.position.x)
        origin_y = float(self.static_map_msg.info.origin.position.y)

        # akadályok rárajzolása
        for (map_x, map_y) in self.obstacle_list:
            self.put_obstacle_to_grid(dynamic_grid_map, map_x, map_y, origin_x, origin_y, resolution)

        # új OccupancyGrid összepakolása
        update_map_with_obstacle = OccupancyGrid()
        update_map_with_obstacle.header = self.static_map_msg.header
        update_map_with_obstacle.header.stamp = self.get_clock().now().to_msg()
        update_map_with_obstacle.info = self.static_map_msg.info
        update_map_with_obstacle.data = dynamic_grid_map.flatten().tolist()

        self.map_publisher.publish(update_map_with_obstacle)
        self.get_logger().info('A dinamikus map publikálva az akadállyal......')

    # világkoordinátában megadott akadályt átalakítom OccupancyGrid-re
    def put_obstacle_to_grid(self, dynamic_grid_map, map_x, map_y, origin_x, origin_y, resolution):
        height, width = dynamic_grid_map.shape
        
        # az akadály félméretét alakítom át méterből cella számra
        radius_x_cells = max(1, int(math.ceil(self.obstacle_half_extent_x / resolution)))
        radius_y_cells = max(1, int(math.ceil(self.obstacle_half_extent_y / resolution)))

        # itt alakítom át a  világkoordinátát cella koordinátára
        grid_x = int((map_x - origin_x) / resolution)
        grid_y = int((map_y - origin_y) / resolution)

        # téglalap határait állítom be
        # bal és jobb szélek
        x0 = max(0, grid_x - radius_x_cells)
        x1 = min(width - 1, grid_x + radius_x_cells)
        # alsó és felső szélek
        y0 = max(0, grid_y - radius_y_cells)
        y1 = min(height - 1, grid_y + radius_y_cells)

        if grid_x < 0 or grid_x >= width or grid_y < 0 or grid_y >= height:
            self.get_logger().warn(f'Az akadály y={map_y:.2f}, x={map_x:.2f} a griden kívül esik. (grid_y={grid_y}, grid_x={grid_x}). ')
            return

        dynamic_grid_map[y0:y1 + 1, x0:x1 + 1] = 100


def main(args=None):
    rclpy.init(args=args)
    node = DynamicObstacleToMapUpdate()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
