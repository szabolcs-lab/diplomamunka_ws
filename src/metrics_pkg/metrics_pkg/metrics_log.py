import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PointStamped
from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_point
import rclpy.time
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
import os
import csv
import math
from datetime import datetime
import numpy as np

class MetricsLog(Node):
    def __init__(self):
        super().__init__("metrics_log")
        
        self.declare_parameter("modszer", "ppo_hybrid")
        self.modszer = self.get_parameter("modszer").value
        
        self.declare_parameter("palya", "occupancy_grid_1.csv")
        self.palya = self.get_parameter("palya").value
        
        self.declare_parameter("path_topic", "/planned_path_smoother")
        self.path_topic = self.get_parameter("path_topic").value
        
        self.declare_parameter("odom_topic", "/odom")
        self.odom_topic = self.get_parameter("odom_topic").value
        
        self.declare_parameter("scan_topic", "/scan")
        self.scan_topic = self.get_parameter("scan_topic").value
        
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.cmd_vel_topic = self.get_parameter("cmd_vel_topic").value

        self.declare_parameter("collision_distance", 0.18)
        self.collision_distance = float(self.get_parameter("collision_distance").value)
        
        self.declare_parameter("stop_time_s", 2.0)
        self.stop_time_s = float(self.get_parameter("stop_time_s").value)
        
        self.declare_parameter("start_speed_eps", 0.01)
        self.start_speed_eps = float(self.get_parameter("start_speed_eps").value)
        
        self.declare_parameter("stop_speed_eps", 0.01)
        self.stop_speed_eps = float(self.get_parameter("stop_speed_eps").value)

        self.declare_parameter("turn_weight", 0.5)
        self.turn_weight = float(self.get_parameter("turn_weight").value)
        
        self.declare_parameter("csv_dir", "./metrics_runs")
        self.csv_dir = self.get_parameter("csv_dir").value
        os.makedirs(self.csv_dir, exist_ok=True)

        self.declare_parameter("goal_tolerance", 0.8)
        self.goal_tolerance = float(self.get_parameter("goal_tolerance").value)
        
        self.declare_parameter("need_goal_to_finish", True)
        self.need_goal_to_finish = bool(self.get_parameter("need_goal_to_finish").value)

        self.last_odom = None
        self.last_scan = None
        self.last_path = None
        self.last_cmd = None
        self.run_started = False
        self.finished = False
        self.start_time = None
        self.last_move_time = None
        self.collision_count = 0
        self.collision_samples = 0
        self.in_collision = False
        self.deviation_sum = 0.0
        self.deviation_sq_sum = 0.0
        self.deviation_max = 0.0
        self.deviation_n = 0
        self.previous_robot_map_x = None
        self.previous_robot_map_y = None
        self.start_robot_map_x = None
        self.start_robot_map_y = None
        self.path_length = 0.0
        self.total_samples = 0
        self.stop_samples = 0
        self.danger_distance = 0.8
        self.danger_samples = 0
        self.energy_sum = 0.0
        self.previous_linear_speed_v = None
        self.previous_angular_speed_w = None

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        
        qos_path = QoSProfile(depth=10)
        qos_path.reliability = ReliabilityPolicy.RELIABLE
        qos_path.durability = DurabilityPolicy.TRANSIENT_LOCAL
        
        self.create_subscription(Path, self.path_topic, self.path_callback, qos_path)
        
        qos_cmd = QoSProfile(depth=20)
        qos_cmd.reliability = ReliabilityPolicy.RELIABLE
        qos_cmd.durability = DurabilityPolicy.VOLATILE
        
        self.create_subscription(Twist, self.cmd_vel_topic, self.cmd_callback, qos_cmd)
        
        qos_scan = QoSProfile(depth=10)
        qos_scan.reliability = ReliabilityPolicy.RELIABLE
        qos_scan.durability = DurabilityPolicy.VOLATILE
        
        self.create_subscription(LaserScan, self.scan_topic, self.scan_callback, qos_scan)
        
        qos_odom = QoSProfile(depth=20)
        qos_odom.reliability = ReliabilityPolicy.RELIABLE
        qos_odom.durability = DurabilityPolicy.VOLATILE

        self.create_subscription(Odometry, self.odom_topic, self.odom_callback, qos_odom)
           

        self.timer = self.create_timer(0.1, self.on_timer)

        self.get_logger().info("MetricsLog elindult....")

    def now_sec(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def odom_callback(self, msg):
        self.last_odom = msg

    def scan_callback(self, msg):
        self.last_scan = msg

    def path_callback(self, msg):
        self.last_path = msg

    def cmd_callback(self, msg):
        self.last_cmd = msg

        speed = abs(msg.linear.x) + abs(msg.angular.z)
        now = self.now_sec()

        if not self.run_started and speed > self.start_speed_eps:
            self.run_started = True
            self.start_time = now
            self.last_move_time = now
            
            return

        if self.run_started and speed > self.stop_speed_eps:
            self.last_move_time = now


    def on_timer(self):

        if self.finished or not self.run_started:      
            return

        if self.last_odom is None or self.last_scan is None:
            return

        robot_map_x, robot_map_y = self.get_robot_pose_from_odom_in_map(self.last_odom)
        if robot_map_x is None:         
            return

        #Start pozíció
        if self.start_robot_map_x is None:
            self.start_robot_map_x = robot_map_x
            self.start_robot_map_y = robot_map_y

        #Megtett út
        if self.previous_robot_map_x is not None:
            delta_x = robot_map_x - self.previous_robot_map_x
            delta_y = robot_map_y - self.previous_robot_map_y
            self.path_length = self.path_length + math.hypot(delta_x, delta_y)

        self.previous_robot_map_x = robot_map_x
        self.previous_robot_map_y = robot_map_y

        #Ütközés
        min_range = self.min_range(self.last_scan)
        in_collision = min_range < self.collision_distance

        if in_collision and not self.in_collision:
            self.collision_count = self.collision_count + 1

        if in_collision:
            self.collision_samples = self.collision_samples +1

        self.in_collision = in_collision

        # Danger
        if min_range < self.danger_distance:
            self.danger_samples = self.danger_samples+ 1

        # Deviation
        if self.last_path is not None and len(self.last_path.poses) >= 2:
            deviation = self.path_deviation(self.last_odom, self.last_path)
            self.deviation_sum = self.deviation_sum + deviation
            self.deviation_sq_sum = self.deviation_sq_sum + (deviation * deviation)
            self.deviation_max = max(self.deviation_max, deviation)
            self.deviation_n = self.deviation_n + 1

        # Stop arány
        self.total_samples = self.total_samples + 1
        if self.last_cmd is not None:
            speed = abs(self.last_cmd.linear.x) + abs(self.last_cmd.angular.z)
            if speed < self.stop_speed_eps:
                self.stop_samples= self.stop_samples + 1

        # Energia
        if self.last_cmd is not None:
            linear_speed_v = self.last_cmd.linear.x
            angular_speed_w = self.last_cmd.angular.z

            if self.previous_linear_speed_v is not None:
                delta_linear_speed_v = abs(linear_speed_v - self.previous_linear_speed_v)
                delta_angular_speed_w = abs(angular_speed_w - self.previous_angular_speed_w)
                
                self.energy_sum = self.energy_sum + (delta_linear_speed_v + self.turn_weight * delta_angular_speed_w)

            self.previous_linear_speed_v = linear_speed_v
            self.previous_angular_speed_w = angular_speed_w

        #Stop feltétel
        now = self.now_sec()
        if (now - self.last_move_time) < self.stop_time_s:
            return

        if self.need_goal_to_finish:
            if self.last_path is None:
                return
            
            distance_goal = self.distance_to_goal(self.last_odom, self.last_path)
            if distance_goal > self.goal_tolerance:
                return

        self.finish_run()


    def finish_run(self):

        self.finished = True
        execute_time = self.now_sec() - self.start_time
        mean_deviation = self.deviation_sum / self.deviation_n if self.deviation_n > 0 else 0.0
        rms_deviation = math.sqrt(self.deviation_sq_sum / self.deviation_n) if self.deviation_n > 0 else 0.0
        stop_ratio = self.stop_samples / self.total_samples if self.total_samples > 0 else 0.0
        danger_ratio = self.danger_samples / self.total_samples if self.total_samples > 0 else 0.0
        collision_duration = self.collision_samples * 0.1

        # kerülő arány
        if self.start_robot_map_x is not None and self.last_path is not None:
            goal_map_x = self.last_path.poses[-1].pose.position.x
            goal_map_y = self.last_path.poses[-1].pose.position.y
            
            straight = math.hypot(goal_map_x - self.start_robot_map_x, goal_map_y - self.start_robot_map_y)
        else:
            straight = 0.0


        if straight > 1e-6:
            path_deviation_ratio = self.path_length / straight
        else:
            path_deviation_ratio = 0.0


        if self.collision_count == 0:
            success = 1 
        else:
            success = 0

        out_path = os.path.join(self.csv_dir, "osszes_eredmeny.csv")
        write_header = not os.path.exists(out_path)

        if write_header:
            with open(out_path, "w", newline="") as f:
                w = csv.writer(f)
                w.writerow(["ido_belyeg", "modszer", "palya","vegrehajtasi_ido","utkozesek","sikeres","atlag_elteres",
                            "max_elteres","rms_palyaelteres","megtett_ut","kerulo_arany","stop_arany","danger_arany","utkozes_idotartam (sec)","energia"])

        with open(out_path, "a", newline="") as f:
            w = csv.writer(f)
            w.writerow([datetime.now().strftime("%Y-%m-%d %H:%M:%S"), self.modszer,self.palya, f"{execute_time:.3f}",
                        self.collision_count,success,f"{mean_deviation:.4f}",f"{self.deviation_max:.4f}", f"{rms_deviation:.4f}", f"{self.path_length:.3f}",
                        f"{path_deviation_ratio:.3f}",f"{stop_ratio:.3f}",f"{danger_ratio:.3f}",f"{collision_duration:.3f}",f"{self.energy_sum:.4f}"] )

        self.get_logger().debug("METRICS MENTVE......")
        rclpy.shutdown()

    def min_range(self, scan):
        ranges = np.array(scan.ranges, dtype=np.float32)
        ranges = np.where(np.isfinite(ranges), ranges, 999.0)
        
        return float(np.min(ranges)) if ranges.size > 0 else 999.0


    def path_deviation(self, odom, path):
        robot_map_x, robot_map_y = self.get_robot_pose_from_odom_in_map(odom)
        if robot_map_x is None:
            return 0.0

        best_min_distance = float('inf')
        
        for pose_stamped in path.poses:
            path_x = pose_stamped.pose.position.x
            path_y = pose_stamped.pose.position.y
            delta_x = path_x - robot_map_x
            delta_y = path_y - robot_map_y
            
            euclides_distance = math.hypot(delta_x, delta_y)    
            best_min_distance = min(best_min_distance, euclides_distance)
                
        return best_min_distance
    

    def distance_to_goal(self, odom, path):
        robot_map_x, robot_map_y = self.get_robot_pose_from_odom_in_map(odom)
        if robot_map_x is None:
            return 0.0

        goal_map_x = path.poses[-1].pose.position.x
        goal_map_y = path.poses[-1].pose.position.y
        
        return math.hypot(goal_map_x - robot_map_x, goal_map_y - robot_map_y)


    def get_robot_pose_from_odom_in_map(self, odom):
        
        odom_point = PointStamped()
        odom_point.header.frame_id = odom.header.frame_id
        odom_point.header.stamp = odom.header.stamp

        odom_point.point.x = odom.pose.pose.position.x
        odom_point.point.y = odom.pose.pose.position.y
        odom_point.point.z = 0.0

        try:
            transform = self.tf_buffer.lookup_transform("map", odom_point.header.frame_id, rclpy.time.Time())
            map_point = do_transform_point(odom_point, transform)
            
            return map_point.point.x, map_point.point.y
        
        except Exception as e:
            self.get_logger().error(f"TF hiba van: {e} !!!!!")
            return None, None


def main(args=None):
    rclpy.init(args=args)
    node = MetricsLog()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()