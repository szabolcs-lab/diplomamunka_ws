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
        
        self.get_logger().info('D* Lite Path Planner node indul....')
        
        self.declare_parameter('margin', 0.68)
        
        self.declare_parameter('resample_step', 0.1)
        self.step = self.get_parameter('resample_step').get_parameter_value().double_value
        
        self.declare_parameter('map_file', 'unknown.csv')
        self.map_file = self.get_parameter('map_file').get_parameter_value().string_value
        
        self.declare_parameter('scenario', 'static')  
        self.scenario = self.get_parameter('scenario').get_parameter_value().string_value
        
        
        self.start = (199, 0)
        self.goal  = (2, 198) #(0, 199)
        self.grid = None  
        self.map_info = None 
        self.planner = None
        self.last_path_msg = None
        self.static_metrics_logged = False
        self.dynamic_replan_logged = False
        
        #a cpu az eslő hívásnál mindig 0%, ezért egyszer meg kell hívni
        self.process_obj = psutil.Process(os.getpid())
        self.process_obj.cpu_percent(interval=None)
        
        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        
        self.map_subscription = self.create_subscription(OccupancyGrid, 'map', self.map_callback, qos)
         
        self.path_publisher = self.create_publisher(Path, '/planned_path_dilated', qos)
        self.path_debug_publisher = self.create_publisher(Path, '/dstar_debug_path', qos) #ez az alapútvonal miatt kell, hogy lássuk
        
        self.timer = self.create_timer(3.0, self.republish_path) #0.5
             
        self.package_dir = os.path.expanduser('~/diplomamunka_ws/src/d_star_lite_pkg')
        self.metrics_log_dir = os.path.join(self.package_dir,'metrics_log')
        os.makedirs(self.metrics_log_dir, exist_ok=True)
                 
        self.metrics_log_file = os.path.join(self.metrics_log_dir, 'd_star_lite_metrics_log.csv')
        
        if not os.path.exists(self.metrics_log_file):
            with open(self.metrics_log_file, 'w', newline= '') as f:
                writer = csv.writer(f)
                writer.writerow(['inditas_idopont', 'algoritmus', 'palya_nev', 'scenario', 'fazis', 'tervezesi_ido (sec)', 'tervezett_ut_hossza (meter)', 'memoria (MB)', 'cpu_kihasznaltsag (%)', 'szamitasok_szama (db)'])

        
        self.get_logger().info('D* Lite Path Planner node inicializálva....')
     
    # mindig a legutólsó útvonalat publikálja újra 
    def republish_path(self):
        if self.last_path_msg is not None:
            self.path_publisher.publish(self.last_path_msg)
            self.path_debug_publisher.publish(self.last_path_msg) 
    
    def map_callback(self, msg):
        
        try:        
            self.get_logger().info(f"Grid: {msg.info.width}x{msg.info.height}, resolution={msg.info.resolution:.3f}")
            
            # időt állítok a log fájlba kell
            timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S')

            # OccupancyGrid-ből csinálunk egy numpy tömböt
            grid_raw = np.array(msg.data).reshape((msg.info.height, msg.info.width))
            grid_binary = (grid_raw > 50).astype(np.int8)

            # az akadályok körül csinálunk egy biztonsági zónát
            margin_m = self.get_parameter('margin').get_parameter_value().double_value
            cells_radius = max(1, int(math.ceil(margin_m / float(msg.info.resolution))))
            grid_padding_extend = self.obstacles_padding_extends(grid_binary, cells_radius)
            
            if self.planner is None:
                self.grid = grid_padding_extend.copy()
                 
                self.get_logger().info("D* Lite indul")
                self.planner = DStarLite(self.grid, self.start, self.goal)
                
                #mennyi idő alatt találja meg az optimális útvonalat
                t0 = time.perf_counter() 
                self.planner.compute_shortest_path()
                t1 = time.perf_counter()
                planning_time = t1 - t0 
                  
                self.get_logger().info("D* Lite  lefutott...")
                
                #elkérem az optimális útvonalat
                path_cells = self.planner.get_path()
                self.get_logger().info(f"planned cells: {len(path_cells)}")
                
                path_length = 0.0
                if path_cells:
                    path_length = self.path_publish(path_cells, msg.info)
                else:
                    self.get_logger().warn("Nem talált útvonalat a dilatált rácson.")
                
                #elkérem a cpu és ram használatot    
                used_ram, cpu_percent = self.measure_resources()
                
                if not self.static_metrics_logged:
                    with open(self.metrics_log_file, 'a', newline='') as f:
                        writer = csv.writer(f)
                        writer.writerow([timestamp, 'D_Star_Lite', self.map_file, self.scenario, 'statikus', planning_time, path_length, used_ram, cpu_percent, self.planner.processed_nodes])
                    self.static_metrics_logged = True
                    
                return
            
            # dinamiku szkenárió
            new_grid = grid_padding_extend    
            different_cells = (self.grid != new_grid)
            ys, xs = np.where(different_cells)
            different_count = len(ys)
            
            self.get_logger().info(f"D* Lite dinamikus változatában a változott cellák száma:: {different_count}")

            if different_count == 0:
                self.get_logger().info("Nincs változás a griden.....")
                return

            for row_y, column_x in zip(ys, xs):
                # ha az adott cella a griden akadály, akkor true értéket ad vissza
                is_obstacle = (new_grid[row_y, column_x] == 1)
                # D* Lite belső gridjét frissítem és újra számolom az akadály miatt az érintett cellákat
                self.planner.update_obstacle((row_y, column_x), is_obstacle)
            
            # az aktuális gridet frissítem a legújabb változásokkal    
            self.grid = new_grid.copy()

            #akadály miatt újratervezés
            self.get_logger().info("Indul a D* Lite újratervezése.....")
            t0 = time.perf_counter()
            self.planner.compute_shortest_path()
            t1 = time.perf_counter()
            planning_time = t1 - t0
            self.get_logger().info(f"A D* Lite újratervezése befejeződött, dt={planning_time:.4f}s")

            #elkérem az új útvonalat
            path_cells = self.planner.get_path()
            
            if not path_cells:
                self.get_logger().warn("Dinamikus adály után nem talált új útvonalat.....")
                return

            # publikálom az új útvonalat és kiszámolom a hosszát
            path_length = self.path_publish(path_cells, msg.info)
            
            if self.scenario == 'dynamic' and not self.dynamic_replan_logged:
                # cpu és ram használat
                used_ram, cpu_percent = self.measure_resources()
                
                with open(self.metrics_log_file, 'a', newline='') as f:
                        writer = csv.writer(f)
                        writer.writerow([timestamp, 'D_Star_Lite', self.map_file, self.scenario, 'dinamikus', planning_time, path_length, used_ram, cpu_percent, self.planner.processed_nodes])
                self.dynamic_replan_logged = True
                
        except Exception as e:
            self.get_logger().error(f"map_callback hiba: {e}\n{traceback.format_exc()}")
            
    # átalakítjuk az útvonalat és elküldjük 
    def path_publish(self, path_cells, map_info):
        points = []
        
        resolution = float(map_info.resolution)
        origin_x  = float(map_info.origin.position.x)
        origin_y  = float(map_info.origin.position.y)

        # átkonvertálom a grid pontokat világ koordinátákká
        for (row_y, column_x) in path_cells:
            world_x = origin_x + (column_x + 0.5) * resolution
            world_y = origin_y + (row_y + 0.5) * resolution
            points.append((world_x, world_y))

        points = self.generate_point_with_fix_spaceing(points, step=self.step)
        
        path_length = 0.0
        for i in range(len(points) -1):
            x0, y0 = points[i]
            x1, y1 = points[i+1]
            path_length = path_length +math.sqrt((x1 - x0)**2 + (y1 - y0)**2)

        path_msg = Path()
        path_msg.header.frame_id = 'map'
        path_msg.header.stamp = self.get_clock().now().to_msg()

        # összerakjuk az útvonalat
        for world_x, world_y in points:
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = world_x
            pose.pose.position.y = world_y
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)

        self.path_publisher.publish(path_msg)
        
        # eltárolom a path_msg-t, hogy később is újra tudjam küldeni RViz-nek az alapútvonalat
        self.last_path_msg = path_msg

        #debug topicon 
        self.path_debug_publisher.publish(self.last_path_msg)
        
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
    node = DStarLitePathPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()
        