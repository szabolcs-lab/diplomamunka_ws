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
import psutil # CPU és RAM -hoz kell
import time
from datetime import datetime
import traceback


class AStarPathPlanner(Node):
    def __init__(self):
        super().__init__('a_star_path_planner')
        
        self.get_logger().info('A* Path Planner node indul....')
        
        self.declare_parameter('margin', 0.5)
        
        self.declare_parameter('resample_step', 0.1)
        self.step = self.get_parameter('resample_step').get_parameter_value().double_value
        
        self.declare_parameter('map_file', 'unknown.csv')
        self.map_file = self.get_parameter('map_file').get_parameter_value().string_value  
        
        self.declare_parameter('scenario', 'static')
        self.scenario = self.get_parameter('scenario').get_parameter_value().string_value
        
         
        self.start = (199, 0)
        self.goal  = (2, 198)#(0, 199)   (4, 197)
        self.grid = None 
        self.path_computed = False
        self.last_path = None    
        self.initial_path_length = 0.0
        self.initial_grid = None  
        self.initial_metrics_logged  = False    
        self.dynamic_replan_logged = False
        self.dynamic_stop_logged = False 
        
        #a cpu az eslő hívásnál mindig 0%, ezért egyszer meg kell hívni
        self.process_obj = psutil.Process(os.getpid())
        self.process_obj.cpu_percent(interval=None)
        
        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        
        self.map_subscription = self.create_subscription(OccupancyGrid, 'map', self.map_callback, qos)
        
        self.path_publisher = self.create_publisher(Path, '/planned_path_dilated', qos)
        
        self.package_dir = os.path.expanduser('~/diplomamunka_ws/src/a_star_pkg')
        self.metrics_log_dir = os.path.join(self.package_dir,'metrics_log')
        os.makedirs(self.metrics_log_dir, exist_ok=True)
               
        self.metrics_log_file = os.path.join(self.metrics_log_dir, 'a_star_metrics_log.csv')
        
        if not os.path.exists(self.metrics_log_file):
            with open(self.metrics_log_file, 'w', newline= '') as f:
                writer = csv.writer(f)
                writer.writerow(['inditas_idopont', 'algoritmus', 'palya_nev', 'scenario', 'fazis', 'tervezesi_ido (sec)', 'tervezett_ut_hossza (meter)', 'memoria (MB)', 'cpu_kihasznaltsag (%)', 'szamitasok_szama (db)'])
                
        self.get_logger().info('A* Path Planner node inicializálva....')
        
    
    def map_callback(self, msg):
             
        try:
            self.get_logger().info(f"Grid: {msg.info.width}x{msg.info.height}, resolution={msg.info.resolution:.3f}")
            
            # idő log fájlba
            timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
            
            # OccupancyGrid-ből csinálok egy numpy tömböt
            grid = np.array(msg.data).reshape((msg.info.height, msg.info.width))
            grid_bin = (grid > 50).astype(np.int8)

            # az akadályok körül csinálunk egy extend paddinget
            margin_m = self.get_parameter('margin').get_parameter_value().double_value
            cells_radius = max(1, int(math.ceil(margin_m / float(msg.info.resolution))))       
            grid_padding_extend = self.obstacles_padding_extends(grid_bin, cells_radius)
            
            # intitial útvonal
            if self.initial_grid is None:
                self.get_logger().info("A* indul..... (initial)")
                
                #itt állítjuk be a kezdő gridet, innentől a következő callbacknél már nem lesz None
                self.initial_grid = grid_padding_extend.copy()
                
                planner = AStar(self.initial_grid, self.start, self.goal)
                
                #mennyi idő alatt találja meg az optimális útvonalat
                t0 = time.perf_counter()
                path_cells = planner.a_star_plan() # A* útkeresését
                t1 = time.perf_counter()
                planning_time = t1 - t0
                
                self.get_logger().info("A*  lefutott..... (initial)")
                self.get_logger().info(f"planned cells: {len(path_cells)}")
                
                path_length = 0.0
                if path_cells:
                    self.last_path = path_cells
                    path_length = self.path_publish(path_cells, msg.info)
                    self.initial_path_length = path_length
                else:
                    self.get_logger().warn("A* nem talált útvonalat... (initial)")
                
                # lekérem a cpu és ram használatot
                used_ram, cpu_percent = self.measure_resources()
                
                if not self.initial_metrics_logged:
                    map_name_for_log = f"{self.map_file}_{self.scenario}"
                    with open(self.metrics_log_file, 'a', newline='') as f:
                        writer = csv.writer(f)
                        writer.writerow([timestamp, 'A_star', map_name_for_log, self.scenario, 'statikus', planning_time, path_length, used_ram, cpu_percent, planner.processed_nodes])
                    self.initial_metrics_logged = True
                
                return
        
            # statikus szkenárió
            if self.scenario == 'static':
                #ha van útvonal, akkor az utulsót mindig újrapublikálom
                if self.last_path is not None:
                    self.get_logger().info("Statikus változat, a last_path úrjapublikálása.....")
                    self.path_publish(self.last_path, msg.info)
                else:
                    self.get_logger().warn("Statikus változat, a last_path üres!!!")
                return
            
            # dinamiku szkenárió
            different_cells = (self.initial_grid != grid_padding_extend)
            ys, xs = np.where(different_cells)
            different_count = len(ys)
            
            self.get_logger().info(f"A* dinamikus változatában a változott cellák száma: {different_count}")
            
            #az eredeti path-t publikálom, mert nincs újratervezés
            if self.last_path is not None:
                self.path_publish(self.last_path, msg.info)
            
            if different_count == 0:
                return
            
            # ha van változás a griden, akkor egyszer logolok dynamic-stop-ot
            if not self.dynamic_stop_logged:
                self.get_logger().info("A* dinamikus változatban a Map megváltozott, nincs újratervezés A* megáll.....")
                
                # lekérem a cpu és ram használatot
                used_ram, cpu_percent = self.measure_resources()
                map_name_for_log = f"{self.map_file}_{self.scenario}"
                
                with open(self.metrics_log_file, 'a', newline='') as f:
                    writer = csv.writer(f)
                    writer.writerow([timestamp, 'A_star', map_name_for_log, self.scenario,'dinamikus-stop',0.0, self.initial_path_length, used_ram, cpu_percent, 0])
                
                self.dynamic_stop_logged = True
            
        except Exception as e:
            self.get_logger().error(f"map_callback hiba: {e}\n{traceback.format_exc()}")
            
    
    # átalakítjuk az útvonalat és elküldjük        
    def path_publish(self, path_cells, map_info):
        points = []
              
        resolution = float(map_info.resolution)
        origin_x  = float(map_info.origin.position.x)
        origin_y  = float(map_info.origin.position.y)

        # átkonvertáljuk a grid pontokat világ koordinátákká
        for (row_y, column_x) in path_cells: 
            world_x = origin_x + (column_x + 0.5) * resolution
            world_y = origin_y + (row_y + 0.5) * resolution
            points.append((world_x, world_y))

        points = self.generate_point_with_fix_spaceing(points, step=self.step)
        
        path_length = 0.0
        for i in range(len(points) -1):
            x0, y0 = points[i]
            x1, y1 = points[i+1]
            path_length =path_length + math.sqrt((x1 - x0)**2 + (y1 - y0)**2)

        path_msg = Path()
        path_msg.header.frame_id = 'map'
        path_msg.header.stamp = self.get_clock().now().to_msg()

        # összerakom az útvonalat
        for world_x, world_y in points:
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = world_x
            pose.pose.position.y = world_y
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)

        self.path_publisher.publish(path_msg)
        self.get_logger().info(f'Az út publikálása befejeződött.....')
        
        return path_length
    
      
    #bővítem az akadályokat egy adott sugarú körrel
    def obstacles_padding_extends(self, grid, radius_cells):
        map_height, map_width = grid.shape
        padding_extends_grid= grid.copy()
        
        obstacle_rows, obstacle_cols = np.where(grid == 1)
        radius_squared = radius_cells ** 2

        for obstacle_row, obstacle_col in zip(obstacle_rows, obstacle_cols):
            up_row = max(0, obstacle_row - radius_cells)
            down_row = min(map_height, obstacle_row + radius_cells + 1)
            left_col = max(0, obstacle_col - radius_cells)
            right_col = min(map_width, obstacle_col + radius_cells + 1)

            for row in range(up_row, down_row):
                delta_row_y = row - obstacle_row

                for column in range(left_col, right_col):
                    delta_column_x = column - obstacle_col

                    if delta_column_x**2 + delta_row_y**2 <= radius_squared:
                        padding_extends_grid[row, column] = 1

        return padding_extends_grid 
        
  
    # két pont között felosztom az útvonalat egyenletes step távolságra
    def generate_point_with_fix_spaceing(self, path_points, step = None):
        if step is None:
            step = getattr(self, 'step', 0.1)
        
        if len(path_points) < 2:
            return path_points[:]
        
        generate_points = [path_points[0]] 
        distance_remainder_last_step = 0.0 
        
        # szakaszokon végigmegyek
        for i in range(len(path_points) - 1):
            start_x, start_y = path_points[i]
            end_x, end_y = path_points[i + 1]
            
            # vektora  hossza
            segment_delta_x = end_x - start_x
            segment_deltay = end_y - start_y
            segment_length = math.hypot(segment_delta_x, segment_deltay)
            
            if segment_length < 1e-9:
                continue
                
            # irány egységvektora
            unit_vector_x = segment_delta_x / segment_length
            unit_vector_y = segment_deltay / segment_length
            
            # első lépés távolsága maradékból indul
            distance_along_segment = step - distance_remainder_last_step
            
            #új pontokat generálok ezen a szakaszon
            while distance_along_segment <= segment_length:
                #új pont pozíciója a szakaszon
                new_point_x = start_x + unit_vector_x * distance_along_segment
                new_point_y = start_y + unit_vector_y * distance_along_segment
                generate_points.append((new_point_x, new_point_y))
                
                distance_along_segment = distance_along_segment + step  # következő lépés
            
            # maradék távolság frissítése a következő szakaszhoz
            distance_remainder_last_step = segment_length - (distance_along_segment - step)
        
        last_x, last_y = generate_points[-1]
        target_x, target_y = path_points[-1]
        
        if math.hypot(last_x - target_x, last_y - target_y) > 1e-6:
            generate_points.append(path_points[-1])
        
        return generate_points
    
    #aktuális memóriahasználat (MB) és CPU (%)
    def measure_resources(self):
        
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
        