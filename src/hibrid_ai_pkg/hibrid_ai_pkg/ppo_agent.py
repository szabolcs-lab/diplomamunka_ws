#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

import numpy as np
import math
from collections import deque

from geometry_msgs.msg import Twist
from nav_msgs.msg import Path, OccupancyGrid
from sensor_msgs.msg import LaserScan

import tf2_ros

from .ppo_training import PPOTraining


def wrap_pi(a: float) -> float:
    return (a + math.pi) % (2.0 * math.pi) - math.pi


class PPOAgent(Node):
    def __init__(self):
        super().__init__('ppo_agent')
        self.get_logger().info('🧠 PPOAgent (PP + stop&turn avoid + scan-slowdown) indul...')

        # ---- params ----
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'chassis')
        self.declare_parameter('cmd_topic', '/cmd_vel')
        self.declare_parameter('path_topic', '/planned_path_dilated')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('map_topic', '/map')

        self.declare_parameter('control_dt', 0.05)

        # path follow tuning
        self.declare_parameter('lookahead_min', 0.30)
        self.declare_parameter('lookahead_max', 0.80)
        self.declare_parameter('goal_tolerance', 0.35)

        # cmd limits
        self.declare_parameter('v_min', 0.06)
        self.declare_parameter('v_max', 0.25)
        self.declare_parameter('w_max', 2.0)

        # PP gains
        self.declare_parameter('pp_kp', 2.0)
        self.declare_parameter('pp_turn_slow_rad', 0.80)

        # scan slowdown (PP-ben is!)
        self.declare_parameter('scan_slow_enter', 1.6)   # ✅ kisebb, ne “túl messziről”
        self.declare_parameter('scan_slow_stop', 0.60)
        self.declare_parameter('hard_stop_min_scan', 0.25)

        # ✅ STOP&TURn avoid (front-only)
        self.declare_parameter('avoid_front_enter', 0.90)   # ✅ ez alatt kezdjük a “tényleges” avoidot
        self.declare_parameter('avoid_front_exit',  1.20)   # hiszterézis
        self.declare_parameter('avoid_turn_gain', 1.2)      # w = gain*w_max
        self.declare_parameter('avoid_turn_max', 1.4)       # rad/s clamp (külön w_max-tól)

        # FRONT/LEFT/RIGHT ablakok
        self.declare_parameter('front_window_deg', 25.0)
        self.declare_parameter('side_window_deg', 35.0)     # bal/jobb szektor mérete
        self.declare_parameter('side_center_deg', 60.0)     # bal/jobb szektor közepe

        # safety / episode
        self.declare_parameter('collision_dist', 0.20)
        self.declare_parameter('episode_timeout_steps', 2500)

        # debug
        self.declare_parameter('debug_print_hz', 1.0)
        self.declare_parameter('use_pure_pursuit_fallback', True)

        # ---- read params ----
        self.map_frame = self.get_parameter('map_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.cmd_topic = self.get_parameter('cmd_topic').value
        self.path_topic = self.get_parameter('path_topic').value
        self.scan_topic = self.get_parameter('scan_topic').value
        self.map_topic = self.get_parameter('map_topic').value

        self.control_dt = float(self.get_parameter('control_dt').value)
        self.lookahead_min = float(self.get_parameter('lookahead_min').value)
        self.lookahead_max = float(self.get_parameter('lookahead_max').value)
        self.goal_tol = float(self.get_parameter('goal_tolerance').value)

        self.v_min = float(self.get_parameter('v_min').value)
        self.v_max = float(self.get_parameter('v_max').value)
        self.w_max = float(self.get_parameter('w_max').value)

        self.pp_kp = float(self.get_parameter('pp_kp').value)
        self.pp_turn_slow_rad = float(self.get_parameter('pp_turn_slow_rad').value)

        self.scan_slow_enter = float(self.get_parameter('scan_slow_enter').value)
        self.scan_slow_stop = float(self.get_parameter('scan_slow_stop').value)
        self.hard_stop_min_scan = float(self.get_parameter('hard_stop_min_scan').value)

        self.avoid_front_enter = float(self.get_parameter('avoid_front_enter').value)
        self.avoid_front_exit = float(self.get_parameter('avoid_front_exit').value)
        self.avoid_turn_gain = float(self.get_parameter('avoid_turn_gain').value)
        self.avoid_turn_max = float(self.get_parameter('avoid_turn_max').value)

        self.front_window_deg = float(self.get_parameter('front_window_deg').value)
        self.side_window_deg = float(self.get_parameter('side_window_deg').value)
        self.side_center_deg = float(self.get_parameter('side_center_deg').value)

        self.collision_dist = float(self.get_parameter('collision_dist').value)
        self.timeout_steps = int(self.get_parameter('episode_timeout_steps').value)

        self.debug_hz = float(self.get_parameter('debug_print_hz').value)
        self.use_fallback = bool(self.get_parameter('use_pure_pursuit_fallback').value)

        # PPO (később)
        self.ppo_trainer = PPOTraining(state_dim=15, action_dim=2, save_dir="./ppo_models", min_update_steps=256)

        # TF2
        self.tf_buffer = tf2_ros.Buffer(cache_time=rclpy.duration.Duration(seconds=10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # state
        self.path_xy = []
        self.have_path = False
        self.pose_map = np.array([0.0, 0.0, 0.0], dtype=np.float32)
        self.have_pose = False
        self.scan = None

        self.map_np = None
        self.map_info = None

        self.vel_history = deque(maxlen=5)

        self.step_in_episode = 0
        self.episode = 0

        self._avoid_mode = False
        self._last_warn_sec = -1
        self._last_debug_ns = 0

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.path_sub = self.create_subscription(Path, self.path_topic, self.path_callback, qos)
        self.scan_sub = self.create_subscription(LaserScan, self.scan_topic, self.scan_callback, qos)
        self.map_sub = self.create_subscription(OccupancyGrid, self.map_topic, self.map_callback, qos)
        self.cmd_pub = self.create_publisher(Twist, self.cmd_topic, qos)

        self.timer = self.create_timer(self.control_dt, self.loop)

        self.get_logger().info(
            f"✅ ready | lookahead=[{self.lookahead_min:.2f},{self.lookahead_max:.2f}] "
            f"avoid_front_enter={self.avoid_front_enter:.2f} scan_slow_enter={self.scan_slow_enter:.2f} "
            f"front_window={self.front_window_deg:.1f}deg v_max={self.v_max:.2f}"
        )

    # ---- callbacks ----
    def path_callback(self, msg: Path):
        if not msg.poses:
            self.have_path = False
            self.path_xy = []
            return
        self.path_xy = [(float(p.pose.position.x), float(p.pose.position.y)) for p in msg.poses]
        self.have_path = True

    def scan_callback(self, msg: LaserScan):
        self.scan = msg

    def map_callback(self, msg: OccupancyGrid):
        self.map_info = msg.info
        try:
            h = int(msg.info.height)
            w = int(msg.info.width)
            self.map_np = np.array(msg.data, dtype=np.int16).reshape((h, w))
        except Exception:
            self.map_np = None

    # ---- warn throttle ----
    def warn_1hz(self, text: str):
        now_sec = int(self.get_clock().now().nanoseconds / 1e9)
        if now_sec != self._last_warn_sec:
            self._last_warn_sec = now_sec
            self.get_logger().warn(text)

    # ---- grid helpers ----
    def world_to_grid(self, x, y):
        if self.map_info is None:
            return None
        res = float(self.map_info.resolution)
        ox = float(self.map_info.origin.position.x)
        oy = float(self.map_info.origin.position.y)
        gx = int(math.floor((x - ox) / res))
        gy = int(math.floor((y - oy) / res))
        return gx, gy

    def grid_in_bounds(self, gx, gy):
        if self.map_info is None:
            return False
        return 0 <= gx < int(self.map_info.width) and 0 <= gy < int(self.map_info.height)

    def cell_cost(self, gx, gy):
        if self.map_np is None or not self.grid_in_bounds(gx, gy):
            return None
        return int(self.map_np[gy, gx])

    # ---- TF pose ----
    def update_pose_map(self) -> bool:
        try:
            tf = self.tf_buffer.lookup_transform(self.map_frame, self.base_frame, rclpy.time.Time())
            t = tf.transform.translation
            q = tf.transform.rotation
            x = float(t.x)
            y = float(t.y)
            siny_cosp = 2.0 * (q.w * q.z)
            cosy_cosp = 1.0 - 2.0 * (q.z * q.z)
            yaw = math.atan2(siny_cosp, cosy_cosp)
            self.pose_map = np.array([x, y, yaw], dtype=np.float32)
            self.have_pose = True
            return True
        except Exception as e:
            self.have_pose = False
            self.warn_1hz(f"❗ TF not ready: {self.map_frame} <- {self.base_frame} | {e}")
            return False

    # ---- scan helpers ----
    def _sector_min(self, center_deg: float, half_width_deg: float) -> float:
        """Min range a megadott szektorban (deg)."""
        if self.scan is None or not self.scan.ranges:
            return 10.0
        ang_min = float(self.scan.angle_min)
        ang_inc = float(self.scan.angle_increment)
        n = len(self.scan.ranges)
        if n < 5 or abs(ang_inc) < 1e-9:
            return 10.0

        c = math.radians(center_deg)
        w = math.radians(half_width_deg)
        a0 = c - w
        a1 = c + w

        i0 = int(max(0, math.floor((a0 - ang_min) / ang_inc)))
        i1 = int(min(n - 1, math.ceil((a1 - ang_min) / ang_inc)))

        best = 10.0
        for i in range(i0, i1 + 1):
            r = self.scan.ranges[i]
            if np.isfinite(r) and 0.0 < r < best:
                best = float(r)
        return best

    def get_front_min(self) -> float:
        return self._sector_min(0.0, self.front_window_deg)

    def get_left_min(self) -> float:
        return self._sector_min(+self.side_center_deg, self.side_window_deg)

    def get_right_min(self) -> float:
        return self._sector_min(-self.side_center_deg, self.side_window_deg)

    def get_min_scan_global(self) -> float:
        if self.scan is None or not self.scan.ranges:
            return 10.0
        vals = [r for r in self.scan.ranges if np.isfinite(r) and r > 0.0]
        return float(min(vals)) if vals else 10.0

    def scan_speed_scale(self, d_front: float) -> float:
        if d_front >= self.scan_slow_enter:
            return 1.0
        if d_front <= self.scan_slow_stop:
            return 0.0
        return float((d_front - self.scan_slow_stop) / max(self.scan_slow_enter - self.scan_slow_stop, 1e-6))

    # ---- path helpers ----
    def closest_path_index_and_dist(self):
        if not self.have_path or not self.have_pose:
            return None, None
        x, y = float(self.pose_map[0]), float(self.pose_map[1])
        d2 = [(px - x) ** 2 + (py - y) ** 2 for (px, py) in self.path_xy]
        i = int(np.argmin(d2))
        return i, float(math.sqrt(d2[i]))

    def find_target_index(self):
        if not self.have_path or len(self.path_xy) < 2 or not self.have_pose:
            return None
        x, y = float(self.pose_map[0]), float(self.pose_map[1])
        d2 = [(px - x) ** 2 + (py - y) ** 2 for (px, py) in self.path_xy]
        closest_idx = int(np.argmin(d2))

        for i in range(closest_idx + 1, min(closest_idx + 120, len(self.path_xy))):
            px, py = self.path_xy[i]
            dist = math.hypot(px - x, py - y)
            if self.lookahead_min <= dist <= self.lookahead_max:
                return i

        return min(closest_idx + 10, len(self.path_xy) - 1)

    # ---- controller ----
    def fallback_cmd(self, target_idx):
        x, y, yaw = map(float, self.pose_map)
        tx, ty = self.path_xy[target_idx]
        ang_to = math.atan2(ty - y, tx - x)
        heading_err = wrap_pi(ang_to - yaw)

        dmin_global = self.get_min_scan_global()
        d_front = self.get_front_min()
        d_left = self.get_left_min()
        d_right = self.get_right_min()

        v_scan_scale = self.scan_speed_scale(d_front)

        # ✅ hiszterézis: csak FRONT alapján
        if self._avoid_mode:
            if d_front > self.avoid_front_exit:
                self._avoid_mode = False
        else:
            # ✅ csak akkor lépj avoid-ba, ha tényleg “előtted” közel van valami
            # (és nem egy enyhe kanyar/oldalfal miatt)
            if d_front < self.avoid_front_enter:
                self._avoid_mode = True

        if self._avoid_mode:
            # STOP & TURN: ne induljon el random irányba
            v = 0.0

            # melyik oldal szabadabb?
            # ha bal oldalt több hely → balra fordulj, különben jobbra
            turn_dir = +1.0 if d_left > d_right else -1.0
            w = float(turn_dir * min(self.avoid_turn_max, self.avoid_turn_gain * self.w_max))

            mode = f"STOP_TURN(L={d_left:.2f},R={d_right:.2f},F={d_front:.2f})"
            return v, w, mode, heading_err, v_scan_scale, dmin_global, d_front, d_left, d_right

        # PP normál
        w = float(np.clip(self.pp_kp * heading_err, -self.w_max, self.w_max))

        v_turn_scale = 1.0 - min(abs(heading_err) / max(self.pp_turn_slow_rad, 1e-6), 1.0)
        v_turn_scale = max(0.15, v_turn_scale)

        v = self.v_max * v_turn_scale * max(0.10, v_scan_scale)
        v = float(np.clip(v, self.v_min if v_scan_scale > 0.05 else 0.0, self.v_max))

        mode = "PP"
        return v, w, mode, heading_err, v_scan_scale, dmin_global, d_front, d_left, d_right

    # ---- debug ----
    def debug_print(self, idx, mode_str, heading_err, v_scan_scale, dmin_global, d_front, d_left, d_right):
        if self.debug_hz <= 0:
            return
        now_ns = self.get_clock().now().nanoseconds
        period_ns = int(1e9 / self.debug_hz)
        if (now_ns - self._last_debug_ns) < period_ns:
            return
        self._last_debug_ns = now_ns

        x, y, yaw = map(float, self.pose_map)
        closest_i, dev = self.closest_path_index_and_dist()
        tx, ty = self.path_xy[idx]

        rg = self.world_to_grid(x, y)
        tg = self.world_to_grid(tx, ty)

        r_cost = self.cell_cost(*rg) if rg else None
        t_cost = self.cell_cost(*tg) if tg else None

        if closest_i is not None:
            cx, cy = self.path_xy[closest_i]
            cg = self.world_to_grid(cx, cy)
            c_cost = self.cell_cost(*cg) if cg else None
        else:
            cx = cy = None
            cg = None
            c_cost = None

        gx, gy = self.path_xy[-1]
        gg = self.world_to_grid(gx, gy)
        g_cost = self.cell_cost(*gg) if gg else None

        delta_it = None if closest_i is None else (idx - closest_i)

        self.get_logger().info(
            f"DBG[{mode_str}] pose_map=({x:.2f},{y:.2f},{yaw:.2f}) "
            f"grid={rg} cell={r_cost} | "
            f"closest_i={closest_i} dev={None if dev is None else f'{dev:.2f}'} "
            f"closest_pt=({None if cx is None else f'{cx:.2f}'},{None if cy is None else f'{cy:.2f}'}) "
            f"cgrid={cg} ccell={c_cost} | "
            f"target_i={idx} (Δi={delta_it}) target_map=({tx:.2f},{ty:.2f}) grid={tg} cell={t_cost} | "
            f"heading_err={heading_err:.2f} v_scan_scale={v_scan_scale:.2f} | "
            f"dmin_global={dmin_global:.2f} front={d_front:.2f} left={d_left:.2f} right={d_right:.2f} | "
            f"goal_grid={gg} goal_cell={g_cost}"
        )

    # ---- done ----
    def check_done(self, idx):
        if self.get_min_scan_global() < self.collision_dist:
            return True, "collision"
        gx, gy = self.path_xy[-1]
        x, y = float(self.pose_map[0]), float(self.pose_map[1])
        if math.hypot(gx - x, gy - y) < self.goal_tol and idx > int(0.98 * (len(self.path_xy) - 1)):
            return True, "goal"
        if self.step_in_episode >= self.timeout_steps:
            return True, "timeout"
        return False, ""

    # ---- loop ----
    def loop(self):
        if not self.update_pose_map():
            self.publish_zero()
            self.step_in_episode += 1
            return

        if not self.have_path:
            self.warn_1hz("❗ No path yet")
            self.publish_zero()
            self.step_in_episode += 1
            return

        if self.scan is None:
            self.warn_1hz("❗ No scan yet")
            self.publish_zero()
            self.step_in_episode += 1
            return

        idx = self.find_target_index()
        if idx is None:
            self.warn_1hz("❗ No target idx")
            self.publish_zero()
            self.step_in_episode += 1
            return

        dmin_global = self.get_min_scan_global()
        if dmin_global < self.hard_stop_min_scan:
            self.warn_1hz(f"🚨 HARD STOP min_scan={dmin_global:.2f} < {self.hard_stop_min_scan:.2f}")
            self.publish_zero()
            self.step_in_episode += 1
            return

        if self.use_fallback:
            v_cmd, w_cmd, mode_str, heading_err, v_scan_scale, dmin_g, d_front, d_left, d_right = self.fallback_cmd(idx)
            done, reason = self.check_done(idx)

            tw = Twist()
            tw.linear.x = float(v_cmd)
            tw.angular.z = float(w_cmd)
            self.cmd_pub.publish(tw)

            self.step_in_episode += 1

            if self.step_in_episode % 50 == 0:
                self.get_logger().info(
                    f"[{mode_str}] step={self.step_in_episode} idx={idx}/{len(self.path_xy)} "
                    f"v={v_cmd:.2f} w={w_cmd:.2f} dmin_global={dmin_g:.2f} front={d_front:.2f} L={d_left:.2f} R={d_right:.2f}"
                )

            self.debug_print(idx, mode_str, heading_err, v_scan_scale, dmin_g, d_front, d_left, d_right)

            if done:
                self.publish_zero()
                self.get_logger().warn(f"🏁 EP END: {reason} | ep={self.episode} steps={self.step_in_episode}")
                self.episode += 1
                self.step_in_episode = 0
                self.vel_history.clear()
                self._avoid_mode = False
            return

        self.publish_zero()
        self.step_in_episode += 1

    def publish_zero(self):
        self.cmd_pub.publish(Twist())


def main(args=None):
    rclpy.init(args=args)
    node = PPOAgent()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publish_zero()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
