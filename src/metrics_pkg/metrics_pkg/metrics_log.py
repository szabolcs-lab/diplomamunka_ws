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


class MetricsLog(Node):
    """
      Start: amikor előszöör megmozdul a robot (/cmd_vel nem 0)
      End: ha 2 másodpercig áll (cmd_vel ~ 0)
      Metrikák:
          vegrehajtasi_ido
          utkozesek_szama (min_range < collision_distance, élváltással számolva)
          path deviation (robot - legközelebbi path pont)
          energia (|dv| + k*|dw|)
    """

    def __init__(self):
        super().__init__("metrics_log")

        #paraméterek
        self.declare_parameter("modszer", "ppo_hybrid")  # astar / dstar / rrtstar / ppo_hybrid
        self.declare_parameter("palya", "occupancy_grid_1.csv")

        self.declare_parameter("path_topic", "/planned_path_smoother")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")

        self.declare_parameter("collision_distance", 0.18)
        self.declare_parameter("stop_time_s", 2.0)          # ennyi ideig áll - vége
        self.declare_parameter("start_speed_eps", 0.01)     # ekkora sebességnél már "indul"
        self.declare_parameter("stop_speed_eps", 0.01)      # ekkora alatt "áll"

        self.declare_parameter("turn_weight", 0.5)          # energia: |dv| + turn_weight*|dw|
        self.declare_parameter("csv_dir", "./metrics_runs")

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

        self.create_subscription(Odometry, self.odom_topic, self.odom_callback, 20)
        self.create_subscription(LaserScan, self.scan_topic, self.scan_callback, 10)
        self.create_subscription(Path, self.path_topic, self.path_callback, 10)
        self.create_subscription(Twist, self.cmd_vel_topic, self.cmd_callback, 20)

        self.timer = self.create_timer(0.1, self.on_timer)

        self.get_logger().info(f"MetricsLog indul: {self.modszer}, palya={self.palya}")
        self.get_logger().info(f"Start: /cmd_vel mozgás, End: {self.stop_time_s}s állás")

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

        # még nem indult
        if not self.run_started:
            return

        # kell minimum odom+scan a metrikákhoz
        if self.last_odom is None or self.last_scan is None:
            return

        # 1) ütközés számlálás
        min_range = self.min_range(self.last_scan)
        in_collision_now = (min_range < self.collision_distance)
        
        if in_collision_now and (not self._in_collision):
            self.collision_count += 1
            
        self._in_collision = in_collision_now

        # 2) deviation
        if self.last_path is not None and len(self.last_path.poses) >= 2:
            path_deviation_distance = self.path_deviation(self.last_odom, self.last_path)
            self.deviation_sum += path_deviation_distance
            self.deviation_max = max(self.deviation_max, path_deviation_distance)
            self.deviation_n += 1

        # 3) energia (cmd_vel változás)
        if self.last_cmd is not None:
            robot_speed = float(self.last_cmd.linear.x)
            robot_turn_speed = float(self.last_cmd.angular.z)
            
            if self.previous_robot_speed is not None and self.previous_robot_turn_speed is not None:
                linear_speed_change = abs(robot_speed - self.previous_robot_speed)
                angular_speed_change = abs(robot_turn_speed - self.previous_robot_turn_speed)
                self.energy_sum += (linear_speed_change + self.turn_weight * angular_speed_change)
                
            self.previous_robot_speed = robot_speed
            self.previous_robot_turn_speed = robot_turn_speed

        # 4) vége feltétel: stop_time_s ideje nem mozgott
        now = self.now_sec()
        if self.last_move_time is not None and (now - self.last_move_time) > self.stop_time_s:
            self.finish_run()

    def min_range(self, scan: LaserScan) -> float:
        ranges = np.array(scan.ranges, dtype=np.float32)
        ranges = np.where(np.isfinite(ranges), ranges, 999.0)
        
        if ranges.size == 0:
            return 999.0
        
        return float(np.min(ranges))

    def path_deviation(self, odom: Odometry, path: Path) -> float:
        robot_x = float(odom.pose.pose.position.x)
        robot_y = float(odom.pose.pose.position.y)

        best = 1e18
        for path_point in path.poses:
            path_point_x = float(path_point.pose.position.x)
            path_point_y = float(path_point.pose.position.y)
            squared_distance = (path_point_x - robot_x) ** 2 + (path_point_y - robot_y) ** 2
            
            if squared_distance < best:
                best = squared_distance
                
        return float(math.sqrt(best)) if best < 1e18 else 0.0

    def finish_run(self):
        self.finished = True

        end_time = self.now_sec()
        exec_time = float(end_time - self.start_time) if self.start_time is not None else 0.0

        mean_dev = (self.deviation_sum / self.deviation_n) if self.deviation_n > 0 else 0.0

        # mentés CSV-be
        out_path = os.path.join(self.csv_dir, "osszes_eredmeny.csv")
        if not os.path.exists(out_path):
            with open(out_path, "w", newline="") as f:
                w = csv.writer(f)
                w.writerow(["ido_belyeg", "modszer", "palya", "vegrehajtasi_ido (sec)", "utkozesek_szama",
                            "atlagos_eltetes (meter)", "max_eltetes (meter)", "sebessegvaltozas_energia"])

        with open(out_path, "a", newline="") as f:
            w = csv.writer(f)
            w.writerow([datetime.now().strftime("%Y-%m-%d %H:%M:%S"), self.modszer, self.palya,f"{exec_time:.3f}",
                        int(self.collision_count), f"{mean_dev:.4f}", f"{self.deviation_max:.4f}", f"{self.energy_sum:.4f}",])

        self.get_logger().info(
            f"[METRICS END] time={exec_time:.2f}s coll={self.collision_count} "
            f"dev_mean={mean_dev:.3f} dev_max={self.deviation_max:.3f} energy={self.energy_sum:.3f}"
        )

        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = MetricsLog()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
