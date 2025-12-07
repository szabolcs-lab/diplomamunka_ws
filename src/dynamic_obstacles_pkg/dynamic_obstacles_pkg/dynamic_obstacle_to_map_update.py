import math
import os

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped

import numpy as np


class DynamicObstacleToMapUpdate(Node):
    def __init__(self):
        super().__init__('dynamic_obstacle_to_map_update')

        # paraméterek beolvasása launch-ból
        #  default érték, ha a launchból nemjön paraméter map
        self.static_map_topic = self.declare_parameter('static_map_topic', 'map').value

        # default érték, ha a launchból nemjön paraméter map_dynamic
        self.dynamic_map_topic = self.declare_parameter('dynamic_map_topic', 'map_dynamic').value

        # fél hosszak (méterben) X és Y irányban
        self.obstacle_half_extent_x = self.declare_parameter('obstacle_half_extent_x', 0.25).value
        self.obstacle_half_extent_y = self.declare_parameter('obstacle_half_extent_y', 1.25).value

        self.get_logger().info(f'Statikus map topicja: {self.static_map_topic}, dinamikus map topicja: {self.dynamic_map_topic}')

        # ide mentjük az utoljára kapott statikus OccupancyGridet
        self.static_map_msg = None 
        
        # itt fogjuk tráolni az akadályokat
        self.obstacle_list = []          

        qos_map = QoSProfile(depth=1) # Buffer mérete 1
        qos_map.reliability = ReliabilityPolicy.RELIABLE # ezzel garantáljuk, hogy minden üzenet megérkezzen
        qos_map.durability = DurabilityPolicy.TRANSIENT_LOCAL # azok a node-ok, amelyek későn csatlakoznak azok is meg fogják kapni a legutolsó üzenetet
        
        qos_obstacle = QoSProfile(depth=10) # Buffer mérete 10
        qos_obstacle.reliability = ReliabilityPolicy.RELIABLE # ezzel garantáljuk, hogy minden üzenet megérkezzen
        qos_obstacle.durability = DurabilityPolicy.TRANSIENT_LOCAL # azok a node-ok, amelyek későn csatlakoznak azok is meg fogják kapni a legutolsó üzenetet

        # feliratkozunk a statikus mapre és a map_callback függvényt meghívjuk
        self.create_subscription(OccupancyGrid, self.static_map_topic, self.map_callback, qos_map)

        # feliratkozunk a dynamic_obstacle-re és az obstacle_callback függvényt meghívjuk 
        self.create_subscription(PoseStamped, 'dynamic_obstacle', self.obstacle_callback, qos_obstacle)

        # dinamikus map publikálása a map_dynamic-on
        self.map_pub = self.create_publisher(OccupancyGrid, self.dynamic_map_topic, qos_map)

    # Callbacks
    def map_callback(self, msg: OccupancyGrid):
        # itt mentjük el a legfrissebb statikus map-et
        self.static_map_msg = msg
        self.get_logger().info(f'Statikus map: {msg.info.height}x{msg.info.width}, resolution={msg.info.resolution:.3f}')

        # ha már van akadály, generáljunk azonnal dinamikus mapet
        self.publish_dynamic_map()

    def obstacle_callback(self, msg: PoseStamped):
        
        # megnézzük, hogy az akadály a map frameben van-e
        if msg.header.frame_id not in ['', 'map']:
            self.get_logger().warn(f'Nincs map {msg.header.frame_id}...')

        # Kiszedjükd az akadály pozícióját ami világkoordinátában van
        world_x = msg.pose.position.x
        world_y = msg.pose.position.y

        # hozzáadjuk a listához
        self.obstacle_list.append((world_x, world_y))
        self.get_logger().info(f'A dinamikus akadály a világban az x={world_x:.2f}, y={world_y:.2f} koordinátán van...')

        # az akadály érkezése után új map-et készítünk
        self.publish_dynamic_map()

    # Dinamikus map generálása
    def publish_dynamic_map(self):
        # megnézzük, hogy van-e statikus térkép
        if self.static_map_msg is None:
            self.get_logger().warn('Nincs még statikus map, így nem tudok dinamikus map-et publikálni...')
            return

        # statikus map-et átalakítjuk numpy array-re, a data mező az egy 1D lista
        base = np.array(self.static_map_msg.data, dtype=np.int16)
        height = self.static_map_msg.info.height
        width = self.static_map_msg.info.width
        base = base.reshape((height, width))

        dynamic_grid_map = base.copy()

        # paraméterek ahhoz, hogy a világot át tudjuk konvertálni gridre
        resolution = float(self.static_map_msg.info.resolution)
        origin_x = float(self.static_map_msg.info.origin.position.x)
        origin_y = float(self.static_map_msg.info.origin.position.y)

        # akadályok rárajzolása
        for (wrold_x, world_y) in self.obstacle_list:
            self.apply_obstacle_to_grid(dynamic_grid_map, wrold_x, world_y, origin_x, origin_y, resolution)

        # új OccupancyGrid összeállítása
        out = OccupancyGrid()
        out.header = self.static_map_msg.header
        out.header.stamp = self.get_clock().now().to_msg()
        out.info = self.static_map_msg.info
        out.data = dynamic_grid_map.flatten().tolist()

        self.map_pub.publish(out)
        self.get_logger().info('A dinamikus map publikálva az akadállyal...')

    # világkoordinátában megadott akadályt átalakítjuk OccupancyGrid-re
    def apply_obstacle_to_grid(self, dynamic_grid_map: np.ndarray, world_x: float, world_y: float, origin_x: float, origin_y: float, resolution: float):
        height, width = dynamic_grid_map.shape
        
        # az akadály félméretét alakítjuk át méterből cella számra
        radius_x_cells = max(1, int(math.ceil(self.obstacle_half_extent_x / resolution)))
        radius_y_cells = max(1, int(math.ceil(self.obstacle_half_extent_y / resolution)))

        # itt alakítjuk át a  világkoordinátát cella koordinátára
        grid_x = int((world_x - origin_x) / resolution)
        grid_y = int((world_y - origin_y) / resolution)

        # téglalap határait állítjuk be
        # bal és jobb szélek
        x0 = max(0, grid_x - radius_x_cells)
        x1 = min(width - 1, grid_x + radius_x_cells)
        # alsó és felső szélek
        y0 = max(0, grid_y - radius_y_cells)
        y1 = min(height - 1, grid_y + radius_y_cells)

        # itt ellenőrizzük, hogy az akadály a griden belül van-e
        if grid_x < 0 or grid_x >= width or grid_y < 0 or grid_y >= height:
            self.get_logger().warn(f'Az akadály y={world_y:.2f}, x={world_x:.2f} a griden kívül esik. (grid_y={grid_y}, grid_x={grid_x}). ')
            return

        # 100-as értékkel jelöljük az akadályt
        dynamic_grid_map[y0:y1 + 1, x0:x1 + 1] = 100


def main(args=None):
    rclpy.init(args=args)
    node = DynamicObstacleToMapUpdate()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
