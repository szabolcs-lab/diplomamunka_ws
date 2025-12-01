import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped

import numpy as np
from .a_star import AStar
import math
import os
import csv
import psutil
import time
from datetime import datetime

import traceback


class AStarPathPlanner(Node):
    def __init__(self):
        super().__init__('a_star_path_planner')
        
        self.get_logger().info('A* Path Planner node indul....')
        
        self.declare_parameter('margin', 0.5)
        self.declare_parameter('resample_step', 0.1)
        self.declare_parameter('map_file', 'unknown.csv')
        self.declare_parameter('scenario', 'static')
        
        self.map_file = self.get_parameter('map_file').get_parameter_value().string_value  
        self.scenario = self.get_parameter('scenario').get_parameter_value().string_value
        self.step = self.get_parameter('resample_step').get_parameter_value().double_value
         
        self.start = (199, 0)
        self.goal  = (0, 199)   
        self.grid = None
        
        self.path_computed = False
        self.last_path = None   
           
        self.initial_path_length = 0.0
        self.initial_grid = None
        
        self.initial_metrics_logged  = False     
        self.dynamic_replan_logged = False
        self.dynamic_stop_logged = False 
        
        self.process_obj = psutil.Process(os.getpid())
        self.process_obj.cpu_percent(interval=None)
        
        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        
        self.map_subscription = self.create_subscription(OccupancyGrid, 'map', self.map_callback, qos)   
        self.path_pub = self.create_publisher(Path, 'planned_path_dilated', qos)
        
        
        self.package_dir = os.path.expanduser('~/diplomamunka_ws/src/a_star_pkg')
        self.metrics_log_dir = os.path.join(self.package_dir,'metrics_log')
        os.makedirs(self.metrics_log_dir, exist_ok=True)         
        self.metrics_log_file = os.path.join(self.metrics_log_dir, 'a_star_metrics_log.csv')
        
        if not os.path.exists(self.metrics_log_file):
            with open(self.metrics_log_file, 'w', newline= '') as f:
                writer = csv.writer(f)
                writer.writerow(['inditas_idopont', 'algoritmus', 'palya_nev', 'scenario', 'fazis', 'tervezesi_ido (sec)', 'tervezett_ut_hossza (meter)', 'memoria (MB)', 'cpu_kihasznaltsag (%)', 'szamitasok_szama (db)'])
                
        self.get_logger().info('A* Path Planner node inicializálva....')
        
    
    def map_callback(self, msg: OccupancyGrid):
             
        try:
            self.get_logger().info(f"Map: {msg.info.width}x{msg.info.height}, resolution={msg.info.resolution:.3f}")
            
            timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
            
            # OccupancyGrid -> bináris rács: 1=akadály, 0=szabad
            grid = np.array(msg.data).reshape((msg.info.height, msg.info.width))
            grid_bin = (grid > 50).astype(np.int8)

            margin_m = self.get_parameter('margin').get_parameter_value().double_value
            cells_radius = max(1, int(math.ceil(margin_m / float(msg.info.resolution))))
            grid_dilated = self.dilate_obstacles(grid_bin, cells_radius)
            
            # -------------------- 1) ELSŐ FUTÁS: INITIAL ÚTVONAL --------------------
            if self.initial_grid is None:
                self.get_logger().info("A* compute start (initial)")
                
                self.initial_grid = grid_dilated.copy()
                planner = AStar(self.initial_grid, self.start, self.goal)
                
                t0 = time.perf_counter()
                path_cells = planner.a_star_plan()
                t1 = time.perf_counter()
                planning_time = t1 - t0
                
                self.get_logger().info("A* compute done (initial)")
                self.get_logger().info(f"planned cells: {len(path_cells)}")
                
                path_length = 0.0
                if path_cells:
                    self.last_path = path_cells
                    path_length = self.path_publish(path_cells, msg.info)
                    self.initial_path_length = path_length
                else:
                    self.get_logger().warn("Nem talált útvonalat a dilatált rácson (initial).")
                
                used_ram, cpu_percent = self.measure_resources()
                
                if not self.initial_metrics_logged:
                    map_name_for_log = f"{self.map_file}_{self.scenario}"
                    with open(self.metrics_log_file, 'a', newline='') as f:
                        writer = csv.writer(f)
                        writer.writerow([timestamp, 'A_star', map_name_for_log, self.scenario, 'statikus', planning_time, path_length, used_ram, cpu_percent, planner.processed_nodes])
                    self.initial_metrics_logged = True
                
                return
        
            # -------------------- 2) STATIKUS SCENARIO --------------------
            if self.scenario == 'static':
                if self.last_path is not None:
                    self.get_logger().info("Static scenario: republishing last_path.")
                    self.path_publish(self.last_path, msg.info)
                else:
                    self.get_logger().warn("Static scenario, de last_path None?!")
                return
            
            # -------------------- 3) DINAMIKUS SCENARIO --------------------
            # NEM tervezünk újra A*-ral, csak megnézzük: a mostani map különbözik-e az eredetitől
            diff_mask = (self.initial_grid != grid_dilated)
            ys, xs = np.where(diff_mask)
            diff_count = len(ys)
            
            self.get_logger().info(f"[A* dynamic] Diff cells count (vs initial): {diff_count}")
            
            # mindenképp az eredeti pathot publikáljuk
            if self.last_path is not None:
                self.path_publish(self.last_path, msg.info)
            
            # ha nincs változás, nincs mit logolni pluszban
            if diff_count == 0:
                return
            
            # ha VAN változás a mapben (pl. dinamikus akadály), akkor egyszer logolunk dynamic-stop-ot
            if not self.dynamic_stop_logged:
                self.get_logger().info("[A* dynamic] Map changed, but A* does NOT replan -> dynamic-stop.")
                
                used_ram, cpu_percent = self.measure_resources()
                map_name_for_log = f"{self.map_file}_{self.scenario}"
                
                with open(self.metrics_log_file, 'a', newline='') as f:
                    writer = csv.writer(f)
                    writer.writerow([timestamp, 'A_star', map_name_for_log, self.scenario,'dinamikus-stop',0.0, self.initial_path_length, used_ram, cpu_percent, 0])
                
                self.dynamic_stop_logged = True
            
        except Exception as e:
            self.get_logger().error(f"map_callback failed: {e}\n{traceback.format_exc()}")
            
            
    def path_publish(self, path_cells: list, map_info: OccupancyGrid):
        
        
        res = float(map_info.resolution)
        ox  = float(map_info.origin.position.x)
        oy  = float(map_info.origin.position.y)

        pts = []

        for (ry, cx) in path_cells:  # ry = row (y), cx = col (x)
            wx = ox + (cx + 0.5) * res
            wy = oy + (ry + 0.5) * res
            pts.append((wx, wy))

        pts = self.resample_path(pts, step=self.step)
        
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
        self.get_logger().info(f'Path has been published.')
        
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
        
        for i in range(len(pts) -1):
            x0, y0 = pts[i]
            x1, y1 = pts[i + 1]
            
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
    node = AStarPathPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()
        