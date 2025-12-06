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
import psutil # CPU és RAM kihasználtságának a méréséhez kell
import time
from datetime import datetime

import traceback


class AStarPathPlanner(Node):
    def __init__(self):
        super().__init__('a_star_path_planner')
        
        self.get_logger().info('A* Path Planner node indul....')
        
        # paraméterek beolvasása yaml-ből launch fájlba, majd onnan a változókba
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
        
        # itt tároljuk az első statikus futás során keletkezett eredményeket   
        self.initial_path_length = 0.0
        self.initial_grid = None  
        self.initial_metrics_logged  = False 
        
        # dinamikus futáshoz egyszeri loggolás, hogy változott a térkép    
        self.dynamic_replan_logged = False
        self.dynamic_stop_logged = False 
        
        # itt készítjük elő a cpu és ram mérését; mivel a cpu az eslő hívásnál mindig 0%, ezért egyszer meg kell hívni
        self.process_obj = psutil.Process(os.getpid())
        self.process_obj.cpu_percent(interval=None)
        
        qos = QoSProfile(depth=10) # Buffer méret 10
        qos.reliability = ReliabilityPolicy.RELIABLE # ezzel garantáljuk, hogy egy üzenet sem fog elveszni
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL # amikor egy node később csatlakozik megkapja az utlsó üzenetet
        
        # a node feliratkozik map topic-ra és minden üzenetnél egy map_callback üzenet hívódik meg
        self.map_subscription = self.create_subscription(OccupancyGrid, 'map', self.map_callback, qos)
        # útvonal publikálása   
        self.path_pub = self.create_publisher(Path, 'planned_path_dilated', qos)
        
        # metrics_log könytár létrehozása
        self.package_dir = os.path.expanduser('~/diplomamunka_ws/src/a_star_pkg')
        self.metrics_log_dir = os.path.join(self.package_dir,'metrics_log')
        os.makedirs(self.metrics_log_dir, exist_ok=True) # ha a mappa még nem létezik, akkor létrehozzuk
        
        # itt állítjuk be, hogy hol legyen a fájl az abszólút elérésiúttal         
        self.metrics_log_file = os.path.join(self.metrics_log_dir, 'a_star_metrics_log.csv')
        
        # első indításkor létrejön a a_star_metrics_log.csv fájl a fejlécekkel, ha még nem létezett
        if not os.path.exists(self.metrics_log_file):
            with open(self.metrics_log_file, 'w', newline= '') as f:
                writer = csv.writer(f)
                writer.writerow(['inditas_idopont', 'algoritmus', 'palya_nev', 'scenario', 'fazis', 'tervezesi_ido (sec)', 'tervezett_ut_hossza (meter)', 'memoria (MB)', 'cpu_kihasznaltsag (%)', 'szamitasok_szama (db)'])
                
        self.get_logger().info('A* Path Planner node inicializálva....')
        
    
    def map_callback(self, msg: OccupancyGrid):
             
        try:
            # ellenőrizzük, hogy megvan-e a térkép
            self.get_logger().info(f"Grid: {msg.info.width}x{msg.info.height}, resolution={msg.info.resolution:.3f}")
            
            # időt állítunk, ami, majd a log fájlba kell
            timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
            
            # OccupancyGrid-ből csinálunk egy numpy tömböt, ami bináris rácsokból fog állni: 1 az akadály (>50), 0 a szabad
            grid = np.array(msg.data).reshape((msg.info.height, msg.info.width))
            grid_bin = (grid > 50).astype(np.int8)

            # az akadályok körül csinálunk egy biztonsági zónát, kipárnázzuk, hogy a robot tudjon egy biztosági távolságot tartani
            margin_m = self.get_parameter('margin').get_parameter_value().double_value
            cells_radius = max(1, int(math.ceil(margin_m / float(msg.info.resolution))))       
            grid_dilated = self.dilate_obstacles(grid_bin, cells_radius)
            
            # intitial útvonal
            # ha még nincs kezdő grid
            if self.initial_grid is None:
                self.get_logger().info("A* indul... (initial)")
                
                #itt állítjuk be a kezdő gridet, innentől a következő callbacknél már nem lesz None
                self.initial_grid = grid_dilated.copy()
                # elindítjuk az A* algortimust, beállítjuk a kezdőértékkel
                planner = AStar(self.initial_grid, self.start, self.goal)
                
                # indítunk egy idő mérést, hogy mennyi idő alatt találja meg az optimális útvonalat
                t0 = time.perf_counter()
                path_cells = planner.a_star_plan() # indítjuk az A* útkeresését
                t1 = time.perf_counter()
                planning_time = t1 - t0
                
                self.get_logger().info("A*  lefutott... (initial)")
                self.get_logger().info(f"planned cells: {len(path_cells)}")
                
                path_length = 0.0
                # ha az algoritmus tlált útvonalat, akkor azt elmentjük a last_path-ba
                if path_cells:
                    self.last_path = path_cells
                    path_length = self.path_publish(path_cells, msg.info) # a meghívjuk a path_publish függvényt, ami visszaadja az útvonal hosszát egy változóba
                    self.initial_path_length = path_length # a visszaadott útvonalat beállítjuk
                else:
                    self.get_logger().warn("A* nem talált útvonalat... (initial)") # ha nincs útvonal kiírjuk
                
                # lekérjük a cpu és ram használatot
                used_ram, cpu_percent = self.measure_resources()
                
                # ha még nincs metrikus adat, akkor egyszer kiírjuk egy csv-be, tehát ez csak egyszer fut le
                if not self.initial_metrics_logged:
                    map_name_for_log = f"{self.map_file}_{self.scenario}"
                    with open(self.metrics_log_file, 'a', newline='') as f:
                        writer = csv.writer(f)
                        writer.writerow([timestamp, 'A_star', map_name_for_log, self.scenario, 'statikus', planning_time, path_length, used_ram, cpu_percent, planner.processed_nodes])
                    self.initial_metrics_logged = True
                
                return
        
            # statikus szkenárió
            # ha a paraméter static, amit az elején beolvasunk
            if self.scenario == 'static':
                # és, ha van útvonal, akkor az utulsót mindig újrapublikáljuk
                if self.last_path is not None:
                    self.get_logger().info("Statikus változat, a last_path úrjapublikálása...")
                    self.path_publish(self.last_path, msg.info)
                else:
                    self.get_logger().warn("Statikus változat, a last_path üres!!!")
                return
            
            # dinamiku szkenárió
            # A* természetéből fakadóan nem tervezünk újra, csak megnézzük, hogy a mostani map különbözik-e az eredetitől
            diff_cells = (self.initial_grid != grid_dilated)
            ys, xs = np.where(diff_cells)
            diff_count = len(ys) # naplózzul a változott cellák számát
            
            self.get_logger().info(f"A* dinamikus változatában a változott cellák száma: {diff_count}")
            
            # mindenképp az eredeti path-t publikáljuk, mert nincs újratervezés
            if self.last_path is not None:
                self.path_publish(self.last_path, msg.info)
            
            # ha nincs változás, nincs mit logolni
            if diff_count == 0:
                return
            
            # ha van változás a griden, akkor egyszer logolunk dynamic-stop-ot
            if not self.dynamic_stop_logged:
                self.get_logger().info("A* dinamikus változatban a Map megváltozott, nincs újratervezés A* megáll...")
                
                # lekérjük a cpu és ram használatot
                used_ram, cpu_percent = self.measure_resources()
                map_name_for_log = f"{self.map_file}_{self.scenario}"
                
                # a dinamikus - metrikus adatok kiírása egyszer egy csv-be, tehát ez csak egyszer fut le
                with open(self.metrics_log_file, 'a', newline='') as f:
                    writer = csv.writer(f)
                    writer.writerow([timestamp, 'A_star', map_name_for_log, self.scenario,'dinamikus-stop',0.0, self.initial_path_length, used_ram, cpu_percent, 0])
                
                self.dynamic_stop_logged = True
            
        except Exception as e:
            self.get_logger().error(f"map_callback hiba: {e}\n{traceback.format_exc()}")
            
    
    # átalakítjuk az útvonalat és elküldjük        
    def path_publish(self, path_cells: list, map_info: OccupancyGrid):
              
        resolution = float(map_info.resolution)
        origin_x  = float(map_info.origin.position.x)
        origin_y  = float(map_info.origin.position.y)

        # simított útvonal pontok tárolására használjuk
        points = []

        # átkonvertáljuk a grid pontokat világ koordinátákká
        for (row_y, column_x) in path_cells:  # ry = row (y), cx = col (x)
            world_x = origin_x + (column_x + 0.5) * resolution
            world_y = origin_y + (row_y + 0.5) * resolution
            points.append((world_x, world_y))

        # az útvonal pontjait simítjuk, hogy eggyenletes legyen
        points = self.resample_path(points, step=self.step)
        
        # kiszámítjuk a teljes út hosszát euklidészi távolságként
        path_length = 0.0
        for i in range(len(points) -1):
            x0, y0 = points[i]
            x1, y1 = points[i+1]
            path_length += math.sqrt((x1 - x0)**2 + (y1 - y0)**2)

        # létrehozunk egy Path objektumot, ami az üzenet lesz
        path_msg = Path()
        
        # beállítjuk a headrt a lokálos koordinátarendszerrel és az idővel
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

        # publikáljuk az összerakott útvonalat
        self.path_pub.publish(path_msg)
        self.get_logger().info(f'Az út publikálása befejeződött...')
        
        return path_length # visszaadjuk az út hosszát
    
      
    # akadáloky párnázása       
    def dilate_obstacles(self, grid: np.ndarray, cells_radius: int):
        height, width = grid.shape
        out = grid.copy()
        
        # kiszűrjük az összes akadályt és azok pontjait
        ys, xs = np.where(grid == 1)
        
        # végigmegyünk a kiszűrt pontokon és szélesítjük az akadály területét egy megadott sugárral
        for y, x in zip(ys, xs):
            y0 = max(0, y - cells_radius)
            y1 = min(height, y + cells_radius + 1)
            x0 = max(0, x - cells_radius)
            x1 = min(width, x + cells_radius +1)
            out[y0:y1,x0:x1] = 1
            
        return out
        
    
    # az útvonal pontjait simítjuk, hogy eggyenletes legyen      
    def resample_path(self, points: list, step=0.1):
        if not points:
            return []
        
        out = [points[0]]
        remainder = 0.0
        
        for i in range(len(points) -1):
            x0, y0 = points[i] # aktuális pont
            x1, y1 = points[i + 1] # köbetkező pont
            
            # két pont közötti távolság
            direction_x, direction_y = x1 - x0, y1 - y0
            segment_length = math.sqrt(direction_x**2 + direction_y**2)
            
            # ha ez az előbb kiszámolt távolság nagyon kicsit, akkor lépünk egyet az iterációban
            if segment_length < 1e-9:
                continue
            
            # kiszámoljuk az x és y egységvektort
            unit_vector_x, unit_vector_y = direction_x/segment_length, direction_y/segment_length
            # maradék távolság ami hiányzott az előző szakaszból, hogy pontosan step távolságra tudjunk lépni
            s = step - remainder
            
            # addig csinálunk az adott szakaszban új pontokat, amyg a szegmens hossz nagyobb vagy egyenlő
            while s <= segment_length:
                out.append((x0 + unit_vector_x * s, y0 + unit_vector_y * s))
                s += step
            
            # új maradék távolság az utolsó elhelyezett ponttól a szakasz végéig; ez lesz a következő iteráció elején a maradék    
            remainder = segment_length - (s - step)
        
        # hogy ne legyen levágva az út vége, ezzel garantáljuk, hogy mindig az eredeti célponton legyen a resample vége   
        if out[-1] != points[-1]:
            out.append(points[-1])
            
        return out
    
    # visszaadjuk az aktuális memóriahasználatot (MB) és CPU-t (%)
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
        