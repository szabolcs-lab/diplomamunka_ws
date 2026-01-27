import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped

import numpy as np
from .d_star_lite import DStarLite 
import math
import time

class DStarLitePathPlanner(Node):
    def __init__(self):
        super().__init__('d_star_lite_path_planner....')
        
        self.declare_parameter('margin', 0.5)
        self.declare_parameter('resample_step', 0.1)
        
        self.margin = self.get_parameter('margin').get_parameter_value().double_value
        self.resample_step = self.get_parameter('resample_step').get_parameter_value().double_value
        
        self.start = (199, 0)
        self.goal = (0, 199)
        self.grid = None
        self.planner = None
        
        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        
        self.map_subscription = self.create_subscription(
            OccupancyGrid, 'map', self.map_callback, qos)
        self.path_pub = self.create_publisher(Path, 'planned_path_dilated', qos)
        
        self.get_logger().info('D* Lite Path Planner inicializálva...')
    
    def map_callback(self, msg: OccupancyGrid):
        grid_raw = np.array(msg.data).reshape((msg.info.height, msg.info.width))
        grid_bin = (grid_raw > 50).astype(np.int8)
        
        cells_radius = max(1, int(math.ceil(self.margin / msg.info.resolution)))
        grid_dilated = self.dilate_obstacles(grid_bin, cells_radius)
        
        if self.planner is None:
            self.grid = grid_dilated.copy()
            self.planner = DStarLite(self.grid, self.start, self.goal)
            self.planner.compute_shortest_path()
            
            path_cells = self.planner.get_path()
            if path_cells:
                self.path_publish(path_cells, msg.info)
            return
        
        # Dynamic scenario
        diff_cells = (self.grid != grid_dilated)
        ys, xs = np.where(diff_cells)
        
        if len(ys) > 0:
            for row_y, column_x in zip(ys, xs):
                is_obstacle = (grid_dilated[row_y, column_x] == 1)
                self.planner.update_obstacle((row_y, column_x), is_obstacle)
            
            self.grid = grid_dilated.copy()
            self.planner.compute_shortest_path()
            
            path_cells = self.planner.get_path()
            if path_cells:
                self.path_publish(path_cells, msg.info)
    
    def path_publish(self, path_cells: list, map_info):
        resolution = float(map_info.resolution)
        origin_x = float(map_info.origin.position.x)
        origin_y = float(map_info.origin.position.y)
        
        points = []
        for row_y, column_x in path_cells:
            world_x = origin_x + (column_x + 0.5) * resolution
            world_y = origin_y + (row_y + 0.5) * resolution
            points.append((world_x, world_y))
        
        points = self.resample_path(points)
        path_msg = Path()
        path_msg.header.frame_id = 'map'
        path_msg.header.stamp = self.get_clock().now().to_msg()
        
        for world_x, world_y in points:
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = world_x
            pose.pose.position.y = world_y
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)
        
        self.path_pub.publish(path_msg)
    
    def dilate_obstacles(self, grid: np.ndarray, cells_radius: int):
        height, width = grid.shape
        out = grid.copy()
        ys, xs = np.where(grid == 1)
        
        for y, x in zip(ys, xs):
            y0 = max(0, y - cells_radius)
            y1 = min(height, y + cells_radius + 1)
            x0 = max(0, x - cells_radius)
            x1 = min(width, x + cells_radius + 1)
            out[y0:y1, x0:x1] = 1
        return out
    
    def resample_path(self, points: list, step=None):
        if not points:
            return []
        
        if step is None:
            step = self.resample_step
            
        out = [points[0]]
        remainder = 0.0
        
        for i in range(len(points) - 1):
            x0, y0 = points[i]
            x1, y1 = points[i + 1]
            
            direction_x, direction_y = x1 - x0, y1 - y0
            segment_length = math.sqrt(direction_x**2 + direction_y**2)
            
            if segment_length < 1e-9:
                continue
            
            unit_vector_x = direction_x / segment_length
            unit_vector_y = direction_y / segment_length
            s = step - remainder
            
            while s <= segment_length:
                out.append((x0 + unit_vector_x * s, y0 + unit_vector_y * s))
                s += step
            
            remainder = segment_length - (s - step)
        
        if out[-1] != points[-1]:
            out.append(points[-1])
        return out

def main(args=None):
    rclpy.init(args=args)
    node = DStarLitePathPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
