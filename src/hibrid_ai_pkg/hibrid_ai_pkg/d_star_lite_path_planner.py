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

        self.declare_parameter('margin', 0.68)
        self.margin = self.get_parameter('margin').get_parameter_value().double_value
        
        self.declare_parameter('resample_step', 0.1)
        self.step = self.get_parameter('resample_step').get_parameter_value().double_value

        self.start = (199, 0)
        self.goal = (2, 198)
        self.grid = None
        self.planner = None
        self.last_path_msg = None

        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self.map_subscription = self.create_subscription(OccupancyGrid,'map', self.map_callback, qos)
        self.path_publisher = self.create_publisher(Path, '/planned_path_dilated', qos)
        self.path_debug_publisher = self.create_publisher(Path, '/dstar_debug_path', qos)

        self.timer = self.create_timer(3.0, self.republish_path)

        self.get_logger().info('D* Lite Path Planner node inicializálva......')

    def republish_path(self):
        if self.last_path_msg is not None:
            now = self.get_clock().now().to_msg()
            self.last_path_msg.header.stamp = now
            for p in self.last_path_msg.poses:
                p.header.stamp = now

            self.path_publisher.publish(self.last_path_msg)
            self.path_debug_publisher.publish(self.last_path_msg)

    def map_callback(self, msg):
        try:
            self.get_logger().info(f"Grid: {msg.info.width}x{msg.info.height}, resolution={msg.info.resolution:.3f}")

            grid_raw = np.array(msg.data).reshape((msg.info.height, msg.info.width))
            grid_binary = (grid_raw > 50).astype(np.int8)

            cells_radius = max(1, int(math.ceil(self.margin / float(msg.info.resolution))))
            grid_padding_extend = self.obstacles_padding_extends(grid_binary, cells_radius)

            #Első tervezés
            if self.planner is None:
                self.grid = grid_padding_extend.copy()

                self.get_logger().info("D* Lite indul....")
                self.planner = DStarLite(self.grid, self.start, self.goal)

                self.planner.compute_shortest_path()
                path_cells = self.planner.get_path()

                if path_cells:
                    self.path_publish(path_cells, msg.info)
                else:
                    self.get_logger().warn("Nem talált útvonalat a dilatált rácson!!!!!!")

                return

            #Dinamikus rész
            new_grid = grid_padding_extend.copy()
            different_cells = (self.grid != new_grid)
            ys, xs = np.where(different_cells)
            different_count = len(ys)

            self.get_logger().info(f"D* Lite dinamikus változatában a változott cellák száma: {different_count}")

            if different_count == 0:
                return

            for row_y, column_x in zip(ys, xs):
                is_obstacle = (new_grid[row_y, column_x] == 1)
                self.planner.update_obstacle((int(row_y), int(column_x)), bool(is_obstacle))

            self.grid = new_grid.copy()

            self.get_logger().info("Indul a D* Lite újratervezése.....")
            self.planner.compute_shortest_path()

            path_cells = self.planner.get_path()
            if not path_cells:
                self.get_logger().warn("Dinamikus akadály után nem talált új útvonalat.....")
                return

            self.path_publish(path_cells, msg.info)

        except Exception as e:
            self.get_logger().error(f"map_callback hiba: {e}\n{traceback.format_exc()} !!!!!")

    def path_publish(self, path_cells, map_info):
        points = []
        resolution = float(map_info.resolution)
        origin_x = float(map_info.origin.position.x)
        origin_y = float(map_info.origin.position.y)

        for (row_y, column_x) in path_cells:
            world_x = origin_x + (column_x + 0.5) * resolution
            world_y = origin_y + (row_y + 0.5) * resolution
            points.append((world_x, world_y))

        points = self.generate_point_with_fix_spaceing(points, step=self.step)

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

        self.path_publisher.publish(path_msg)
        self.path_debug_publisher.publish(path_msg)

        self.last_path_msg = path_msg

        self.get_logger().info('Az út publikálása befejeződött...')

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


def main(args=None):
    rclpy.init(args=args)
    node = DStarLitePathPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()