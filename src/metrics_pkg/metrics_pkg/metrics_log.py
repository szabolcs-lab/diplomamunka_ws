import os
import csv
import math
from datetime import datetime

import numpy as np
import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

from geometry_msgs.msg import PointStamped
from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_point
import rclpy.time


class MetricsLog(Node):
    """
    Start: amikor először megmozdul a robot (/cmd_vel nem 0)
    End: cél közelében + stop_time_s ideje áll (cmd_vel ~ 0)
    - végrehajtási idő
    - ütközések száma (min_range < collision_distance, élváltással számolva)
    - path deviation (robot - legközelebbi path pont)
    - energia (|dv| + k*|dw|)
    """

    def __init__(self):
        super().__init__("metrics_log")

        # paraméterek
        self.declare_parameter("modszer", "ppo_hybrid")  # astar / dstar / rrtstar / ppo_hybrid
        self.declare_parameter("palya", "occupancy_grid_1.csv")

        self.declare_parameter("path_topic", "/planned_path_smoother")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")

        self.declare_parameter("collision_distance", 0.18)

        self.declare_parameter("stop_time_s", 2.0)
        self.declare_parameter("start_speed_eps", 0.01)
        self.declare_parameter("stop_speed_eps", 0.01)

        self.declare_parameter("turn_weight", 0.5)
        self.declare_parameter("csv_dir", "./metrics_runs")

        #cél-közeli befejezés
        self.declare_parameter("goal_tolerance", 0.8)          # méter
        self.declare_parameter("need_goal_to_finish", True)    # ha True: csak cél közelében fejezünk be

        # beolvasás
        self.modszer = str(self.get_parameter("modszer").value)
        self.palya = str(self.get_parameter("palya").value)

        self.path_topic = str(self.get_parameter("path_topic").value)
        self.odom_topic = str(self.get_parameter("odom_topic").value)
        self.scan_topic = str(self.get_parameter("scan_topic").value)
        self.cmd_vel_topic = str(self.get_parameter("cmd_vel_topic").value)

        self.collision_distance = float(self.get_parameter("collision_distance").value)

        self.stop_time_s = float(self.get_parameter("stop_time_s").value)
        self.start_speed_eps = float(self.get_parameter("start_speed_eps").value)
        self.stop_speed_eps = float(self.get_parameter("stop_speed_eps").value)

        self.turn_weight = float(self.get_parameter("turn_weight").value)
        self.csv_dir = str(self.get_parameter("csv_dir").value)
        os.makedirs(self.csv_dir, exist_ok=True)

        self.goal_tolerance = float(self.get_parameter("goal_tolerance").value)
        self.need_goal_to_finish = bool(self.get_parameter("need_goal_to_finish").value)

        # cache
        self.last_odom = None
        self.last_scan = None
        self.last_path = None
        self.last_cmd = None

        # futás állapot
        self.run_started = False
        self.start_time = None
        self.last_move_time = None
        self.finished = False

        # ütközés
        self.collision_count = 0
        self._in_collision = False

        # deviation
        self.deviation_sum = 0.0
        self.deviation_max = 0.0
        self.deviation_n = 0

        # energia (sebességváltozás)
        self.energy_sum = 0.0
        self.previous_robot_speed = None
        self.previous_robot_turn_speed = None

        # sub-ok
        self.create_subscription(Odometry, self.odom_topic, self.odom_callback, 20)
        self.create_subscription(LaserScan, self.scan_topic, self.scan_callback, 10)
        self.create_subscription(Path, self.path_topic, self.path_callback, 10)
        self.create_subscription(Twist, self.cmd_vel_topic, self.cmd_callback, 20)

        self.timer = self.create_timer(0.1, self.on_timer)
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.get_logger().info(f"MetricsLog indul: modszer={self.modszer}, palya={self.palya}")
        self.get_logger().info(f"Start: {self.cmd_vel_topic} mozgás")
        
        if self.need_goal_to_finish:
            self.get_logger().info(f"End: cél közelében (tol={self.goal_tolerance}m) + {self.stop_time_s}s állás")
        else:
            self.get_logger().info(f"End: {self.stop_time_s}s állás (cél nélkül is)")

    def now_sec(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def odom_callback(self, msg: Odometry):
        self.last_odom = msg

    def scan_callback(self, msg: LaserScan):
        self.last_scan = msg

    def path_callback(self, msg: Path):
        self.last_path = msg

    def cmd_callback(self, msg: Twist):
        self.last_cmd = msg

        robot_speed = float(msg.linear.x)
        robot_turn_speed = float(msg.angular.z)
        speed = abs(robot_speed) + abs(robot_turn_speed)

        now = self.now_sec()

        # start feltétel
        if (not self.run_started) and speed > self.start_speed_eps:
            self.run_started = True
            self.start_time = now
            self.last_move_time = now
            self.get_logger().info("[METRICS] Futás indult (cmd_vel mozgás).")
            return

        # ha már fut, frissítjük a "legutóbbi mozgás" időt
        if self.run_started and speed > self.stop_speed_eps:
            self.last_move_time = now

    def on_timer(self):
        if self.finished:
            return

        if not self.run_started:
            return

        if self.last_odom is None or self.last_scan is None:
            return
        
        rx, ry = self.robot_xy_in_map(self.last_odom)
        if rx is None:
            return

        # 1) ütközés számlálás
        min_range = self.min_range(self.last_scan)
        in_collision_now = (min_range < self.collision_distance)
        
        if in_collision_now and (not self._in_collision):
            self.collision_count += 1
        self._in_collision = in_collision_now

        # 2) deviation
        if self.last_path is not None and len(self.last_path.poses) >= 2:
            dev = self.path_deviation(self.last_odom, self.last_path)
            self.deviation_sum += dev
            self.deviation_max = max(self.deviation_max, dev)
            self.deviation_n += 1

        # 3) energia
        if self.last_cmd is not None:
            robot_speed = float(self.last_cmd.linear.x)
            robot_turn_speed = float(self.last_cmd.angular.z)

            if self.previous_robot_speed is not None and self.previous_robot_turn_speed is not None:
                dv = abs(robot_speed - self.previous_robot_speed)
                dw = abs(robot_turn_speed - self.previous_robot_turn_speed)
                self.energy_sum += (dv + self.turn_weight * dw)

            self.previous_robot_speed = robot_speed
            self.previous_robot_turn_speed = robot_turn_speed

        # 4) vége feltétel
        now = self.now_sec()
        stopped_long_enough = (self.last_move_time is not None and (now - self.last_move_time) > self.stop_time_s)

        if not stopped_long_enough:
            return

        # ha kell cél, akkor csak cél közelében zárunk
        if self.need_goal_to_finish:
            if self.last_path is None or len(self.last_path.poses) < 1:
                # nincs path -> nem tudjuk a célt, inkább NE zárjuk le
                return

            dist_goal = self.distance_to_goal(self.last_odom, self.last_path)
            if dist_goal > self.goal_tolerance:
                # áll, de még nincs közel a célhoz - valószínű "beragadt", de most nem akarunk hamis befejezést
                return

        self.finish_run()

    def min_range(self, scan: LaserScan) -> float:
        ranges = np.array(scan.ranges, dtype=np.float32)
        ranges = np.where(np.isfinite(ranges), ranges, 999.0)
        
        if ranges.size == 0:
            return 999.0
        
        return float(np.min(ranges))

    def path_deviation(self, odom: Odometry, path: Path) -> float:
        robot_x, robot_y = self.robot_xy_in_map(odom)
        if robot_x is None:
            return 0.0  # vagy return None és akkor ne számold

        best_sq = 1e18
        for p in path.poses:
            px = float(p.pose.position.x)  # map
            py = float(p.pose.position.y)  # map
            sq = (px - robot_x) ** 2 + (py - robot_y) ** 2
            if sq < best_sq:
                best_sq = sq

        return float(math.sqrt(best_sq)) if best_sq < 1e18 else 0.0

    def distance_to_goal(self, odom: Odometry, path: Path) -> float:
        robot_x, robot_y = self.robot_xy_in_map(odom)
        if robot_x is None:
            return 999.0  # TF nélkül ne zárjon le tévesen

        goal_x = float(path.poses[-1].pose.position.x)
        goal_y = float(path.poses[-1].pose.position.y)

        return float(math.hypot(goal_x - robot_x, goal_y - robot_y))

    def finish_run(self):
        self.finished = True

        end_time = self.now_sec()
        exec_time = float(end_time - self.start_time) if self.start_time is not None else 0.0
        mean_dev = (self.deviation_sum / self.deviation_n) if self.deviation_n > 0 else 0.0

        # mentés CSV-be
        out_path = os.path.join(self.csv_dir, "osszes_eredmeny.csv")
        write_header = (not os.path.exists(out_path))

        if write_header:
            with open(out_path, "w", newline="") as f:
                w = csv.writer(f)
                w.writerow(["ido_belyeg", "modszer", "palya","vegrehajtasi_ido (sec)", "utkozesek_szama", "atlagos_eltetes (meter)", 
                            "max_eltetes (meter)", "sebessegvaltozas_energia"])

        with open(out_path, "a", newline="") as f:
            w = csv.writer(f)
            w.writerow([datetime.now().strftime("%Y-%m-%d %H:%M:%S"), self.modszer,self.palya, f"{exec_time:.3f}", int(self.collision_count),
                        f"{mean_dev:.4f}", f"{self.deviation_max:.4f}",f"{self.energy_sum:.4f}"])

        self.get_logger().info(f"[METRICS END] time={exec_time:.2f}s coll={self.collision_count} dev_mean={mean_dev:.3f} dev_max={self.deviation_max:.3f} energy={self.energy_sum:.3f}")

        rclpy.shutdown()
        
    def robot_xy_in_map(self, odom: Odometry):
        p = PointStamped()
        p.header.frame_id = odom.header.frame_id  # várhatóan "odom"
        p.header.stamp = odom.header.stamp
        p.point.x = float(odom.pose.pose.position.x)
        p.point.y = float(odom.pose.pose.position.y)
        p.point.z = 0.0

        try:
            tf = self.tf_buffer.lookup_transform("map", p.header.frame_id, rclpy.time.Time())
            p_map = do_transform_point(p, tf)
            return float(p_map.point.x), float(p_map.point.y)
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
