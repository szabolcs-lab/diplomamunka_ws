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

        # Paraméterek
        # statikus map topic (ahonnan a map_publication küldi)
        self.static_map_topic = self.declare_parameter('static_map_topic', 'map').value

        # dinamikus map topic (ide publikálunk)
        self.dynamic_map_topic = self.declare_parameter('dynamic_map_topic', 'map_dynamic').value

        # fél hosszak (méterben) X és Y irányban
        self.obstacle_half_extent_x = self.declare_parameter('obstacle_half_extent_x', 0.25).value
        self.obstacle_half_extent_y = self.declare_parameter('obstacle_half_extent_y', 1.25).value

        self.get_logger().info(f'Static map topic: {self.static_map_topic}, dynamic map topic: {self.dynamic_map_topic}')

        # Állapot
        self.static_map_msg = None       # utoljára kapott statikus OccupancyGrid
        self.obstacle_list = []          # (wx, wy) akadályok listája

        # QoS: ugyanaz, mint a map_publication-ben (RELIABLE + TRANSIENT_LOCAL)
        qos_map = QoSProfile(depth=1)
        qos_map.reliability = ReliabilityPolicy.RELIABLE
        qos_map.durability = DurabilityPolicy.TRANSIENT_LOCAL
        
        # QoS a dynamic_obstacle topicra: ugyanaz, mint a spawnerben
        qos_obstacle = QoSProfile(depth=10)
        qos_obstacle.reliability = ReliabilityPolicy.RELIABLE
        qos_obstacle.durability = DurabilityPolicy.VOLATILE

        # Feliratkozás a statikus mapre
        self.create_subscription(OccupancyGrid, self.static_map_topic, self.map_callback, qos_map)

        # Feliratkozás a dinamikus akadályokra
        self.create_subscription(PoseStamped, 'dynamic_obstacle', self.obstacle_callback, qos_obstacle)

        # Dinamikus map publikálása
        self.map_pub = self.create_publisher(OccupancyGrid, self.dynamic_map_topic, qos_map)

    # Callbacks
    def map_callback(self, msg: OccupancyGrid):
        self.static_map_msg = msg
        self.get_logger().info(f'Static map received: {msg.info.width}x{msg.info.height}, res={msg.info.resolution:.3f}')

        # ha már vannak akadályok, generáljunk azonnal dinamikus mapet
        self.publish_dynamic_map()

    def obstacle_callback(self, msg: PoseStamped):
        if msg.header.frame_id not in ['', 'map']:
            self.get_logger().warn(f'dynamic_obstacle frame_id={msg.header.frame_id}, de "map"-ot várok.')

        wx = msg.pose.position.x
        wy = msg.pose.position.y

        self.obstacle_list.append((wx, wy))
        self.get_logger().info(f'Received dynamic obstacle at (x={wx:.2f}, y={wy:.2f}). Currently {len(self.obstacle_list)} obstacle(s).')

        self.publish_dynamic_map()

    # Dinamikus map generálása
    def publish_dynamic_map(self):
        if self.static_map_msg is None:
            self.get_logger().warn('Nincs még statikus map, nem tudok dinamikusat publikálni.')
            return

        # alap: statikus map numpy array-ben
        base = np.array(self.static_map_msg.data, dtype=np.int16)
        h = self.static_map_msg.info.height
        w = self.static_map_msg.info.width
        base = base.reshape((h, w))

        dyn = base.copy()

        # paraméterek a világ->rács konverzióhoz
        res = float(self.static_map_msg.info.resolution)
        ox = float(self.static_map_msg.info.origin.position.x)
        oy = float(self.static_map_msg.info.origin.position.y)

        # akadályok rárajzolása
        for (wx, wy) in self.obstacle_list:
            self.apply_obstacle_to_grid(dyn, wx, wy, ox, oy, res)

        # új OccupancyGrid összeállítása
        out = OccupancyGrid()
        out.header = self.static_map_msg.header
        out.header.stamp = self.get_clock().now().to_msg()
        out.info = self.static_map_msg.info
        out.data = dyn.flatten().tolist()

        self.map_pub.publish(out)
        self.get_logger().info('Published dynamic map with obstacles.')

    def apply_obstacle_to_grid(self, grid: np.ndarray, wx: float, wy: float, ox: float, oy: float, res: float):
        h, w = grid.shape
        
        rx_cells = max(1, int(math.ceil(self.obstacle_half_extent_x / res)))
        ry_cells = max(1, int(math.ceil(self.obstacle_half_extent_y / res)))

        # világ → rács indexek
        cx = int((wx - ox) / res)
        cy = int((wy - oy) / res)

        # tartomány kirakása a gridre
        x0 = max(0, cx - rx_cells)
        x1 = min(w - 1, cx + rx_cells)
        y0 = max(0, cy - ry_cells)
        y1 = min(h - 1, cy + ry_cells)

        # out of bounds esetet kicsit jelezzük
        if cx < 0 or cx >= w or cy < 0 or cy >= h:
            self.get_logger().warn(f'Obstacle (x={wx:.2f}, y={wy:.2f}) a map rácson kívül esik. (cx={cx}, cy={cy}). ')
            return

        # 100-as értékkel jelöljük az akadályt (occupied)
        grid[y0:y1 + 1, x0:x1 + 1] = 100


def main(args=None):
    rclpy.init(args=args)
    node = DynamicObstacleToMapUpdate()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
