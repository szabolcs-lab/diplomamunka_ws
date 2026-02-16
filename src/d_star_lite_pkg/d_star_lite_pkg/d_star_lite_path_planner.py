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
        
        # paraméterek beolvasása yaml-ből launch fájlba, majd onnan a változókba
        self.declare_parameter('margin', 0.8)
        self.declare_parameter('resample_step', 0.1)
        self.declare_parameter('map_file', 'unknown.csv')
        self.declare_parameter('scenario', 'static') 
        
        self.map_file = self.get_parameter('map_file').get_parameter_value().string_value
        self.scenario = self.get_parameter('scenario').get_parameter_value().string_value
        self.step = self.get_parameter('resample_step').get_parameter_value().double_value
        
        self.start = (199, 0)
        self.goal  = (0, 199)
        self.grid = None
        
        self.map_info = None # OccupancyGrid.info elmentve
        self.planner = None # DStarLite példány
        
        self.last_path_msg = None # ez az alapútvonal miatt kell, hogy eltároljuk
        
        
        self.static_metrics_logged = False
        self.dynamic_replan_logged = False
        
        # itt készítjük elő a cpu és ram mérését; mivel a cpu az eslő hívásnál mindig 0%, ezért egyszer meg kell hívni
        self.process_obj = psutil.Process(os.getpid())
        self.process_obj.cpu_percent(interval=None)
        
        qos = QoSProfile(depth=10) # Buffer méret 10
        qos.reliability = ReliabilityPolicy.RELIABLE # ezzel garantáljuk, hogy egy üzenet sem fog elveszni
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL # amikor egy node később csatlakozik megkapja az utlsó üzenetet
        
        # a node feliratkozik map topic-ra és minden üzenetnél egy map_callback üzenet hívódik meg
        self.map_subscription = self.create_subscription(OccupancyGrid, 'map', self.map_callback, qos)
        
        # útvonal publikálása a planned_path_dilated topicon    
        self.path_pub = self.create_publisher(Path, 'planned_path_dilated', qos)
        
        self.path_debug_pub = self.create_publisher(Path, 'dstar_debug_path', qos) #ez az alapútvonal miatt kell, hogy lássuk
        
        # ez az időzítő republish_path hívja meg 0.5 másodpercenként
        self.timer = self.create_timer(0.5, self.republish_path) 
        
        # metrics_log könytár létrehozása     
        self.package_dir = os.path.expanduser('~/diplomamunka_ws/src/d_star_lite_pkg')
        self.metrics_log_dir = os.path.join(self.package_dir,'metrics_log')
        os.makedirs(self.metrics_log_dir, exist_ok=True) # ha a mappa még nem létezik, akkor létrehozzuk
        
        # itt állítjuk be, hogy hol legyen a fájl az abszólút elérésiúttal          
        self.metrics_log_file = os.path.join(self.metrics_log_dir, 'd_star_lite_metrics_log.csv')
        
        # első indításkor létrejön az rrt_star_metrics_log.csv fájl a fejlécekkel, ha még nem létezett
        if not os.path.exists(self.metrics_log_file):
            with open(self.metrics_log_file, 'w', newline= '') as f:
                writer = csv.writer(f)
                writer.writerow(['inditas_idopont', 'algoritmus', 'palya_nev', 'scenario', 'fazis', 'tervezesi_ido (sec)', 'tervezett_ut_hossza (meter)', 'memoria (MB)', 'cpu_kihasznaltsag (%)', 'szamitasok_szama (db)'])

        
        self.get_logger().info('D* Lite Path Planner node inicializálva....')
     
    # mindig a legutólsó útvonalat publikálja újra 
    def republish_path(self):
        if self.last_path_msg is not None:
            self.path_pub.publish(self.last_path_msg)
            self.path_debug_pub.publish(self.last_path_msg) 
    
    def map_callback(self, msg: OccupancyGrid):
        
        try:
            
            # ellenőrizzük, hogy megvan-e a térkép          
            self.get_logger().info(f"Grid: {msg.info.width}x{msg.info.height}, resolution={msg.info.resolution:.3f}")
            
            # időt állítunk, ami, majd a log fájlba kell
            timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S')

            # OccupancyGrid-ből csinálunk egy numpy tömböt, ami bináris rácsokból fog állni: 1 az akadály (>50), 0 a szabad
            grid_raw = np.array(msg.data).reshape((msg.info.height, msg.info.width))
            grid_bin = (grid_raw > 50).astype(np.int8)

            # az akadályok körül csinálunk egy biztonsági zónát, kipárnázzuk, hogy a robot tudjon egy biztosági távolságot tartani
            margin_m = self.get_parameter('margin').get_parameter_value().double_value
            cells_radius = max(1, int(math.ceil(margin_m / float(msg.info.resolution))))
            grid_dilated = self.dilate_obstacles(grid_bin, cells_radius)
            
            if self.planner is None:
                
                #itt állítjuk be, lemásoljuk az eredetit kezdő gridet, innentől a következő callbacknél már nem lesz None
                self.grid = grid_dilated.copy()
                
                # elindítjuk az D* Lite algortimust, beállítjuk a kezdőértékkel    
                self.get_logger().info("D* Lite indul")
                self.planner = DStarLite(self.grid, self.start, self.goal)
                
                # indítunk egy idő mérést, hogy mennyi idő alatt találja meg az optimális útvonalat
                t0 = time.perf_counter() 
                self.planner.compute_shortest_path()
                t1 = time.perf_counter()
                planning_time = t1 - t0 
                  
                self.get_logger().info("D* Lite  lefutott...")
                
                # lekérjük az optimális útvonalat
                path_cells = self.planner.get_path()
                self.get_logger().info(f"planned cells: {len(path_cells)}")
                
                path_length = 0.0
                # ha az algoritmus tlált útvonalat, akkor azt publikáljuk
                if path_cells:
                    path_length = self.path_publish(path_cells, msg.info) # a meghívjuk a path_publish függvényt, ami visszaadja az útvonal hosszát egy változóba
                else:
                    self.get_logger().warn("Nem talált útvonalat a dilatált rácson.")
                
                # lekérjük a cpu és ram használatot    
                used_ram, cpu_percent = self.measure_resources()
                
                # ha még nincs metrikus adat, akkor egyszer kiírjuk egy csv-be, tehát ez csak egyszer fut le
                if not self.static_metrics_logged:
                    with open(self.metrics_log_file, 'a', newline='') as f:
                        writer = csv.writer(f)
                        writer.writerow([timestamp, 'D_Star_Lite', self.map_file, self.scenario, 'statikus', planning_time, path_length, used_ram, cpu_percent, self.planner.processed_nodes])
                    self.static_metrics_logged = True
                    
                return
            
            # dinamiku szkenárió
            # D* Lite dinamikusságának a kihasználáshoz megnézzük, hogy a mostani map különbözik-e az eredetitől
            new_grid = grid_dilated # az új gridet, amin már szerepel az új akadály, hozzáadjuk a new_gride-hez
            # létrehozunk egy logikai mátrix-ot, amelynél true érték szerepek azoknál a celláknál, ahol az eredeti grid és az új _grid eltér      
            diff_cells = (self.grid != new_grid)
            ys, xs = np.where(diff_cells) # visszaadjuk azokat a sor és oszlopindexeit, ahol true az érték
            diff_count = len(ys) # naplózzul a változott cellák számát
            
            self.get_logger().info(f"D* Lite dinamikus változatában a változott cellák száma:: {diff_count}")

            if diff_count == 0:
                self.get_logger().info("Nincs változás a griden...")
                return

            # végigmegyünk a párosított sor és oszlopindexeken
            for row_y, column_x in zip(ys, xs):
                # ha az adott cella a griden akadály, akkor true értéket ad vissza
                is_obstacle = (new_grid[row_y, column_x] == 1)
                # D* Lite belső gridjét frissítjük és újra számoljuk az akadály miatt az érintett cellákat
                self.planner.update_obstacle((row_y, column_x), is_obstacle)
            
            # Az aktuális gridet frissítjük a legújabb változásokkal    
            self.grid = new_grid.copy()

            # akadály miatt újratervezünk
            self.get_logger().info("Indul a D* Lite újratervezése...")
            t0 = time.perf_counter()
            self.planner.compute_shortest_path()
            t1 = time.perf_counter()
            planning_time = t1 - t0
            self.get_logger().info(f"A D* Lite újratervezése befejeződött, dt={planning_time:.4f}s")

            # lekérjük az új útvonalat
            path_cells = self.planner.get_path()
            
            # ha a path_cells üres nem talált új útvonalat
            if not path_cells:
                self.get_logger().warn("Dinamikus adály után nem talált új útvonalat...")
                return

            # publikáljuk az új útvonalat a függvény meghívásával és kiszámoljuk a hosszát
            path_length = self.path_publish(path_cells, msg.info)
            
            # ha dinamikus szkenárió van, lett új út és még ezt nem loggoltuk, metrikák szintjén
            if self.scenario == 'dynamic' and not self.dynamic_replan_logged:
                # lekérjük a cpu és ram használatot
                used_ram, cpu_percent = self.measure_resources()
                
                # kiírjuk az új metrikai adatokat a csv-be
                with open(self.metrics_log_file, 'a', newline='') as f:
                        writer = csv.writer(f)
                        writer.writerow([timestamp, 'D_Star_Lite', self.map_file, self.scenario, 'dinamikus', planning_time, path_length, used_ram, cpu_percent, self.planner.processed_nodes])
                self.dynamic_replan_logged = True
                
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
        for (row_y, column_x) in path_cells:
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
        # beállítjuk a headrt a lokális koordinátarendszerrel és az idővel
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
        
        # eltároljuk a path_msg-t, hogy később is újra tudjuk küldeni RViz-nek az alapútvonalat
        self.last_path_msg = path_msg

        # azonnal elküldjük egyszer a debug topicon is
        self.path_debug_pub.publish(self.last_path_msg)
        
        self.get_logger().info(f'Az út publikálása befejeződött...')
        
        return path_length
       
             
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
    node = DStarLitePathPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()
        