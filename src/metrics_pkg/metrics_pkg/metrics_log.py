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
        self._in_collision = False

        self.deviation_sum = 0.0
        self.deviation_sq_sum = 0.0
        self.deviation_max = 0.0
        self.deviation_n = 0

        self.prev_rx = None
        self.prev_ry = None
        self.start_rx = None
        self.start_ry = None
        self.path_length = 0.0

        self.total_samples = 0
        self.stop_samples = 0

        self.danger_distance = 0.8
        self.danger_samples = 0

        self.energy_sum = 0.0
        self.prev_v = None
        self.prev_w = None

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

        rx, ry = self.robot_xy_in_map(self.last_odom)
        if rx is None:
            
            return

        #Start pozíció
        if self.start_rx is None:
            self.start_rx = rx
            self.start_ry = ry

        #Megtett út
        if self.prev_rx is not None:
            dx = rx - self.prev_rx
            dy = ry - self.prev_ry
            self.path_length += math.sqrt(dx*dx + dy*dy)

        self.prev_rx = rx
        self.prev_ry = ry

        #Ütközés
        min_range = self.min_range(self.last_scan)
        in_collision = min_range < self.collision_distance

        if in_collision and not self._in_collision:
            self.collision_count += 1

        if in_collision:
            self.collision_samples += 1

        self._in_collision = in_collision

        # Danger
        if min_range < self.danger_distance:
            self.danger_samples += 1

        # Deviation
        if self.last_path is not None and len(self.last_path.poses) >= 2:
            dev = self.path_deviation(self.last_odom, self.last_path)
            self.deviation_sum += dev
            self.deviation_sq_sum += dev * dev
            self.deviation_max = max(self.deviation_max, dev)
            self.deviation_n += 1

        # Stop arány
        self.total_samples += 1
        if self.last_cmd is not None:
            speed = abs(self.last_cmd.linear.x) + abs(self.last_cmd.angular.z)
            if speed < self.stop_speed_eps:
                self.stop_samples += 1

        # Energia
        if self.last_cmd is not None:
            v = self.last_cmd.linear.x
            w = self.last_cmd.angular.z

            if self.prev_v is not None:
                dv = abs(v - self.prev_v)
                dw = abs(w - self.prev_w)
                self.energy_sum += dv + self.turn_weight * dw

            self.prev_v = v
            self.prev_w = w

        #Stop feltétel
        now = self.now_sec()
        if (now - self.last_move_time) < self.stop_time_s:
            return

        if self.need_goal_to_finish:
            if self.last_path is None:
                return
            
            dist_goal = self.distance_to_goal(self.last_odom, self.last_path)
            if dist_goal > self.goal_tolerance:
                return

        self.finish_run()


    def finish_run(self):

        self.finished = True
        exec_time = self.now_sec() - self.start_time
        mean_dev = self.deviation_sum / self.deviation_n if self.deviation_n > 0 else 0.0
        rms_dev = math.sqrt(self.deviation_sq_sum / self.deviation_n) if self.deviation_n > 0 else 0.0
        stop_ratio = self.stop_samples / self.total_samples if self.total_samples > 0 else 0.0
        danger_ratio = self.danger_samples / self.total_samples if self.total_samples > 0 else 0.0
        collision_duration = self.collision_samples * 0.1

        # kerülő arány
        if self.start_rx is not None and self.last_path is not None:
            goal_x = self.last_path.poses[-1].pose.position.x
            goal_y = self.last_path.poses[-1].pose.position.y
            straight = math.hypot(goal_x - self.start_rx, goal_y - self.start_ry)
        else:
            straight = 0.0

        detour_ratio = self.path_length / straight if straight > 1e-6 else 0.0

        success = 1 if self.collision_count == 0 else 0
        #finish_reason = "goal" if success else "collision"

        out_path = os.path.join(self.csv_dir, "osszes_eredmeny.csv")
        write_header = not os.path.exists(out_path)

        if write_header:
            with open(out_path, "w", newline="") as f:
                w = csv.writer(f)
                w.writerow(["ido_belyeg", "modszer", "palya","vegrehajtasi_ido","utkozesek","sikeres","atlag_elteres",
                            "max_elteres","rms_palyaelteres","megtett_ut","kerulo_arany","stop_arany","danger_arany","utkozes_idotartam (sec)","energia"])

        with open(out_path, "a", newline="") as f:
            w = csv.writer(f)
            w.writerow([datetime.now().strftime("%Y-%m-%d %H:%M:%S"), self.modszer,self.palya, f"{exec_time:.3f}",
                        self.collision_count,success,f"{mean_dev:.4f}",f"{self.deviation_max:.4f}", f"{rms_dev:.4f}", f"{self.path_length:.3f}",
                        f"{detour_ratio:.3f}",f"{stop_ratio:.3f}",f"{danger_ratio:.3f}",f"{collision_duration:.3f}",f"{self.energy_sum:.4f}"] )

        self.get_logger().debug("METRICS MENTVE......")
        rclpy.shutdown()

    def min_range(self, scan):
        ranges = np.array(scan.ranges, dtype=np.float32)
        ranges = np.where(np.isfinite(ranges), ranges, 999.0)
        
        return float(np.min(ranges)) if ranges.size > 0 else 999.0

    def path_deviation(self, odom, path):
        rx, ry = self.robot_xy_in_map(odom)
        if rx is None:
            return 0.0

        best = 1e9
        for p in path.poses:
            px = p.pose.position.x
            py = p.pose.position.y
            d = math.hypot(px - rx, py - ry)
            if d < best:
                best = d
                
        return best

    def distance_to_goal(self, odom, path):
        rx, ry = self.robot_xy_in_map(odom)
        if rx is None:
            return 999.0

        goal_x = path.poses[-1].pose.position.x
        goal_y = path.poses[-1].pose.position.y
        
        return math.hypot(goal_x - rx, goal_y - ry)

    def robot_xy_in_map(self, odom):
        
        p = PointStamped()
        p.header.frame_id = odom.header.frame_id
        p.header.stamp = odom.header.stamp
        p.point.x = odom.pose.pose.position.x
        p.point.y = odom.pose.pose.position.y
        p.point.z = 0.0

        try:
            tf = self.tf_buffer.lookup_transform("map", p.header.frame_id, rclpy.time.Time())
            p_map = do_transform_point(p, tf)
            
            return p_map.point.x, p_map.point.y
        
        except Exception:
            return None, None


def main(args=None):
    rclpy.init(args=args)
    node = MetricsLog()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()