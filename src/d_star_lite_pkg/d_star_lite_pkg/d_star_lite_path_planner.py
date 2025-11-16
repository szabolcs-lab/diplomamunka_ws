import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped

import numpy as np
from .d_star_lite import DStarLite 
import math
import psutil
import os
import time
import csv
from datetime import datetime

import traceback


class DStarLitePathPlanner(Node):
    def __init__(self):
        super().__init__('d_star_lite_path_planner')
        
        self.get_logger().info('D* Lite Path Planner node initialization....')
        
        self.declare_parameter('margin', 0.5)
        self.declare_parameter('resample_step', 0.1)
        self.declare_parameter('map_file', 'unknown.csv') 
        
        self.start = (199, 0)
        self.goal  = (0, 199)
        self.grid = None
        self.metrics_logged = False
        self.map_file = self.get_parameter('map_file').get_parameter_value().string_value
        
        self.process_obj = psutil.Process(os.getpid())
        self.process_obj.cpu_percent(interval=None)
        
        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        
        self.map_subscription = self.create_subscription(OccupancyGrid, 'map', self.map_callback, qos)    
        self.path_pub = self.create_publisher(Path, 'planned_path_dilated', qos)
        
        self.package_dir = os.path.expanduser('~/diplomamunka_ws/src/d_star_lite_pkg')
        self.metrics_log_dir = os.path.join(self.package_dir,'metrics_log')
        os.makedirs(self.metrics_log_dir, exist_ok=True)         
        self.metrics_log_file = os.path.join(self.metrics_log_dir, 'd_star_lite_metrics_log.csv')
        
        if not os.path.exists(self.metrics_log_file):
            with open(self.metrics_log_file, 'w', newline= '') as f:
                writer = csv.writer(f)
                writer.writerow(['timestamp', 'algorithm', 'map_name', 'planning_time (sec)', 'path_length (meter)', 'memory (MB)', 'cpu_percent (%)', 'computation_load (db)'])
        
        self.get_logger().info('D* Lite Path Planner node has been initialized....')
        
    
    def map_callback(self, msg: OccupancyGrid):
        
        try:
            self.get_logger().info(f"Map arrived: {msg.info.width}x{msg.info.height}, res={msg.info.resolution:.3f}")
            
            timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S')

            grid = np.array(msg.data).reshape((msg.info.height, msg.info.width))
            self.grid = (grid > 50).astype(np.int8)

            margin_m = self.get_parameter('margin').get_parameter_value().double_value
            cells_radius = max(1, int(math.ceil(margin_m / float(msg.info.resolution))))
            self.grid = self.dilate_obstacles(self.grid, cells_radius)
           
            self.get_logger().info("compute start")
            planner = DStarLite(self.grid, self.start, self.goal)
           
            t0 = time.perf_counter() 
            planner.compute_shortest_path()
            t1 = time.perf_counter()
            planning_time = t1 - t0   
            self.get_logger().info("compute done")
            
            path_cells = planner.get_path()
            self.get_logger().info(f"planned cells: {len(path_cells)}")
            
            path_length = 0.0
            
            if path_cells:
                path_length = self.path_publish(path_cells, msg.info)
            else:
                self.get_logger().warn("Nem talált útvonalat a dilatált rácson.")
                
            used_ram, cpu_percent = self.measure_resources()
            
            if not self.metrics_logged:
                with open(self.metrics_log_file, 'a', newline='') as f:
                    writer = csv.writer(f)
                    writer.writerow([timestamp, 'D_Star_Lite', self.map_file, planning_time, path_length, used_ram, cpu_percent, planner.processed_nodes])
                self.metrics_logged = True
                
        except Exception as e:
            self.get_logger().error(f"map_callback failed: {e}\n{traceback.format_exc()}")
            
    
    def path_publish(self, path_cells: list, map_info: OccupancyGrid):
        step = self.get_parameter('resample_step').get_parameter_value().double_value
        
        res = float(map_info.resolution)
        ox  = float(map_info.origin.position.x)
        oy  = float(map_info.origin.position.y)

        pts = []

        for (ry, cx) in path_cells:  # ry = row (y), cx = col (x)
            wx = ox + (cx + 0.5) * res
            wy = oy + (ry + 0.5) * res
            pts.append((wx, wy))

        pts = self.resample_path(pts, step=step)
        
        path_length = 0.0
        for i in range(len(pts) -1):
            x0, y0 = pts[i]
            x1, y1 = pts[i+1]
            path_length += math.sqrt((x1 - x0)**2 + (y1 - y0)**2)

        path_msg = Path()
        path_msg.header.frame_id = 'map'
        path_msg.header.stamp = self.get_clock().now().to_msg()

        for wx, wy in pts:
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = wx
            pose.pose.position.y = wy
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)

        self.path_pub.publish(path_msg)
        self.get_logger().info(
            f'Path has been published. N={len(path_msg.poses)}, '
            f'first=({pts[0][0]:.2f},{pts[0][1]:.2f}), '
            f'last=({pts[-1][0]:.2f},{pts[-1][1]:.2f})'
        )
        
        return path_length
      
             
    def dilate_obstacles(self, grid: np.ndarray, cells_radius: int):
        height, width = grid.shape
        out = grid.copy()
        
        ys, xs = np.where(grid == 1)
        
        for y, x in zip(ys, xs):
            y0 = max(0, y - cells_radius)
            y1 = min(height, y + cells_radius + 1)
            x0 = max(0, x - cells_radius)
            x1 = min(width, x + cells_radius +1)
            out[y0:y1,x0:x1] = 1
            
        return out
  
            
    def resample_path(self, pts, step=0.1):
        if not pts:
            return []
        
        out = [pts[0]]
        acc = 0.0
        
        for i in range(1, len(pts)):
            x0, y0 = pts[i-1]
            x1, y1 = pts[i]
            
            dx, dy = x1 - x0, y1 - y0
            seg = math.sqrt(dx**2 + dy**2)
            
            if seg < 1e-9:
                continue
            
            nx, ny = dx/seg, dy/seg
            s = step - acc
            
            while s <= seg:
                out.append((x0 + nx * s, y0 + ny * s))
                s += step
                
            acc = seg - (s - step)
            
        if out[-1] != pts[-1]:
            out.append(pts[-1])
            
        return out
    
    def measure_resources(self):
        """
        Visszaadja az aktuális memóriahasználatot (MB) és CPU %-ot.
        """
        used_ram = self.process_obj.memory_info().rss / (1024 * 1024)
        cpu_percent = self.process_obj.cpu_percent(interval=None)  # az előző hívás óta eltelt időre

        return used_ram, cpu_percent
        
def main(args=None):
    rclpy.init(args=args)
    node = DStarLitePathPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()
        