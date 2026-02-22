import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped

import numpy as np
from .d_star_lite import DStarLite
import math
import traceback

class DStarLitePathPlanner(Node):
    def __init__(self):
        super().__init__('d_star_lite_path_planner')

        self.get_logger().info('D* Lite Path Planner node indul....')

        self.declare_parameter('margin', 0.65)
        self.declare_parameter('resample_step', 0.1)

        self.margin = self.get_parameter('margin').get_parameter_value().double_value
        self.resample_step = self.get_parameter('resample_step').get_parameter_value().double_value

        self.start = (199, 0)
        self.goal = (2, 198) #(0, 199)
        self.grid = None
        self.planner = None
        self.last_path_msg = None

        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self.map_subscription = self.create_subscription(OccupancyGrid, 'map', self.map_callback, qos)
        self.path_pub = self.create_publisher(Path, 'planned_path_dilated', qos)
        self.path_debug_pub = self.create_publisher(Path, 'dstar_debug_path', qos)
        
        self.timer = self.create_timer(3.0, self.republish_path) #0.5

        self.get_logger().info('D* Lite Path Planner node inicializálva....')

    """
    def republish_path(self):
        if self.last_path_msg is not None:
            self.path_pub.publish(self.last_path_msg)
            self.path_debug_pub.publish(self.last_path_msg)
    """
    def republish_path(self):
        if self.last_path_msg is None:
            return

        #stamp frissítés reset után is jó legyen
        now = self.get_clock().now().to_msg()
        self.last_path_msg.header.stamp = now
        for p in self.last_path_msg.poses:
            p.header.stamp = now

        self.path_pub.publish(self.last_path_msg)
        self.path_debug_pub.publish(self.last_path_msg)


    def map_callback(self, msg: OccupancyGrid):
        try:
            self.get_logger().info(f"Grid: {msg.info.height}x{msg.info.width}, resolution={msg.info.resolution:.3f}")

            
            grid_raw = np.array(msg.data).reshape((msg.info.height, msg.info.width))
            grid_bin = (grid_raw > 50).astype(np.int8)

            
            cells_radius = max(1, int(math.ceil(self.margin / msg.info.resolution)))
            grid_dilated = self.dilate_obstacles(grid_bin, cells_radius)

            if self.planner is None:
                self.grid = grid_dilated.copy()
                self.get_logger().info("D* Lite indul")
                self.planner = DStarLite(self.grid, self.start, self.goal)
                self.planner.compute_shortest_path()

                path_cells = self.planner.get_path()
                self.get_logger().info(f"Első path: {path_cells}")
                
                if path_cells:
                    self.path_publish(path_cells, msg.info)
                else:
                    self.get_logger().warn("Nem talált útvonalat a dilatált rácson.")
                return

            # dinamikus újratervezés
            diferent_cells = (self.grid != grid_dilated)
            diferent_ys, diferent_xs = np.where(diferent_cells)

    
            if len(diferent_ys) == 0:
                self.get_logger().debug("Nincs térképváltozás")
                return

            for row_y, column_x in zip(diferent_ys, diferent_xs):
                is_obstacle = (grid_dilated[row_y, column_x] == 1)
                self.planner.update_obstacle((row_y, column_x), is_obstacle)

            self.grid = grid_dilated.copy()
            
            self.get_logger().info("Indul a D* Lite újratervezése...")
            
            self.planner.compute_shortest_path()
            path_cells = self.planner.get_path()
            
            if path_cells:
                self.path_publish(path_cells, msg.info)
            else:
                self.get_logger().warn("Dinamikus akadály után nem talált új útvonalat...")

        except Exception as e:
            self.get_logger().error(f"map_callback hiba: {e}\n{traceback.format_exc()}")

    def path_publish(self, path_cells: list, map_info):
        resolution = float(map_info.resolution)
        origin_x = float(map_info.origin.position.x)
        origin_y = float(map_info.origin.position.y)

        points = []
        for row_y, column_x in path_cells:
            world_x = origin_x + (column_x + 0.5) * resolution
            world_y = origin_y + (row_y + 0.5) * resolution
            points.append((world_x, world_y))

        points = self.resample_path(points, step=self.resample_step)

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
        self.last_path_msg = path_msg
        self.path_debug_pub.publish(self.last_path_msg)
        
        return

    # akadáloky párnázása 
    def dilate_obstacles(self, grid: np.ndarray, radius_cells: int):
        map_height, map_width = grid.shape
        dilaated_grid = grid.copy()
        
        # kiszűrjük az összes akadályt és azok pontjait
        obstacle_rows, obstacle_cols = np.where(grid == 1)

        #négyzet
        radius_squared = radius_cells ** 2

        # végigmegyünk a kiszűrt pontokon és szélesítjük az akadály területét egy megadott sugárral
        for obstacle_row, obstacle_col in zip(obstacle_rows, obstacle_cols):
            min_row = max(0, obstacle_row - radius_cells)
            max_row = min(map_height, obstacle_row + radius_cells + 1)
            min_col = max(0, obstacle_col - radius_cells)
            max_col = min(map_width, obstacle_col + radius_cells + 1)

            for row in range(min_row, max_row):
                row_offset = row - obstacle_row

                for column in range(min_col, max_col):
                    column_offset = column - obstacle_col

                    #Circle ellenőrzés!
                    if column_offset**2 + row_offset**2 <= radius_squared:
                        dilaated_grid[row, column] = 1

        return dilaated_grid

    """
    def resample_path(self, points: list, step=None):
        if not points:
            return []
        
        if step is None:
            step = self.resample_step
            
        out_result = [points[0]]
        previous_point = points[0]
        
        for actual_point in points[1:]:
            direction_x = actual_point[0] - previous_point[0]
            direction_y = actual_point[1] - previous_point[1]
            
            length = math.sqrt(direction_x**2 + direction_y**2)
            
            if length < 1e-9:
                previous_point = actual_point
                continue
            
            unit_vector_x = direction_x / length
            unit_vector_y = direction_y / length
            
            steps = int(length // step)
            for i in range(1, steps + 1):
                distance = i * step
                out_result.append((previous_point[0] + unit_vector_x * distance, previous_point[1] + unit_vector_y * distance))
            
            out_result.append(actual_point)
            previous_point = actual_point

        return out_result
    """
    def resample_path(self, path_points: list[tuple[float, float]], step: float = None):
        """
        Robotikai útvonal resampling egyenletes távolságraa.
        Minden új pont pontosan 'step' távolságra van egymástól.
        """
        if step is None:
            step = getattr(self, 'step', 0.1)  # self.step vagy alapértelmezett 0.1m
        
        if len(path_points) < 2:
            return path_points[:]
        
        resampled_points = [path_points[0]]  # Kezdőpont mindig benne
        distance_remainder = 0.0  # Hátralévő távolság az előző lépésből
        
        # Minden szakaszon végigmegyünk
        for i in range(len(path_points) - 1):
            # Szakasz kezdő- és végpontja
            start_x, start_y = path_points[i]
            end_x, end_y = path_points[i + 1]
            
            # Szakasz vektora és hossza
            segment_dx = end_x - start_x
            segment_dy = end_y - start_y
            segment_length = math.hypot(segment_dx, segment_dy)  # Euklidészi távolság
            
            if segment_length < 1e-9:  # Túl rövid szakasz, kihagyjuk
                continue
                
            # Irány egységvektora
            unit_vector_x = segment_dx / segment_length
            unit_vector_y = segment_dy / segment_length
            
            # Első lépés távolsága (maradékból indulunk)
            distance_along_segment = step - distance_remainder
            
            # Új pontokat generálunk ezen a szakaszon
            while distance_along_segment <= segment_length:
                # Új pont pozíciója a szakaszon
                new_point_x = start_x + unit_vector_x * distance_along_segment
                new_point_y = start_y + unit_vector_y * distance_along_segment
                resampled_points.append((new_point_x, new_point_y))
                
                distance_along_segment += step  # Következő lépés
            
            # Maradék távolság frissítése a következő szakaszhoz
            distance_remainder = segment_length - (distance_along_segment - step)
        
        # Garantáljuk, hogy a célpont mindig benne legyen
        last_x, last_y = resampled_points[-1]
        target_x, target_y = path_points[-1]
        
        if math.hypot(last_x - target_x, last_y - target_y) > 1e-6:
            resampled_points.append(path_points[-1])
        
        return resampled_points
    
    
def main(args=None):
    rclpy.init(args=args)
    node = DStarLitePathPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
