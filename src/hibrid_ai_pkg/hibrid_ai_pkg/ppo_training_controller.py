import os
import math
import numpy as np
import torch

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

from .ppo_training import PPOTraining

from ament_index_python.packages import get_package_prefix




class PPOTrainingController(Node):
    def __init__(self):
        super().__init__('ppo_training_controller')

        self.get_logger().error("### PPO TRAINING CONTROLLER STARTED (THIS FILE) ###")
        self.get_logger().error(f"### __file__ = {__file__} ###")
        
        self.declare_parameter('control_hz', 10.0)
        
        self.declare_parameter('teacher_topic', '/cmd_vel_nav2')
        self.declare_parameter('cmd_vel_out', '/cmd_vel')

        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('path_topic', '/planned_path_dilated')

        self.declare_parameter('delta_v_max', 0.2)   # m/s
        self.declare_parameter('delta_w_max', 1.)    # rad/s

        hz = float(self.get_parameter('control_hz').value)

        self.teacher_topic = str(self.get_parameter('teacher_topic').value)
        self.cmd_vel_out = str(self.get_parameter('cmd_vel_out').value)

        self.odom_topic = str(self.get_parameter('odom_topic').value)
        self.scan_topic = str(self.get_parameter('scan_topic').value)
        self.path_topic = str(self.get_parameter('path_topic').value)

        self.delta_v_max = float(self.get_parameter('delta_v_max').value)
        self.delta_w_max = float(self.get_parameter('delta_w_max').value)
              
        self.declare_parameter('save_dir', '')  # ha üres: automatikus (portable)

        param_save_dir = str(self.get_parameter('save_dir').value).strip()

        if param_save_dir:
            self.save_dir = os.path.expanduser(param_save_dir)
        else:
            # 1) kiderítjük, hol van telepítve a csomag: <ws>/install/hibrid_ai_pkg
            pkg_prefix = get_package_prefix('hibrid_ai_pkg')

            # 2) ebből kiszámoljuk a workspace gyökerét: <ws>
            ws_root = os.path.dirname(os.path.dirname(pkg_prefix))  # .../install/hibrid_ai_pkg -> .../<ws>

            # 3) ide mentünk
            self.save_dir = os.path.join(ws_root, 'src', 'hibrid_ai_pkg', 'ppo_models')

        os.makedirs(self.save_dir, exist_ok=True)
        self.get_logger().info(f"PPO save_dir (workspace portable): {self.save_dir}")


        # ROS I/O 
        qos = QoSProfile(depth=10)

        self.sub_odom = self.create_subscription(Odometry, self.odom_topic, self.odom_cb, qos)
        self.sub_scan = self.create_subscription(LaserScan, self.scan_topic, self.scan_cb, qos)
        self.sub_path = self.create_subscription(Path, self.path_topic, self.path_cb, qos)
        self.sub_teacher = self.create_subscription(Twist, self.teacher_topic, self.teacher_cb, qos)

        self.pub_cmd = self.create_publisher(Twist, self.cmd_vel_out, qos)

        # PPO 
        self.ppo = PPOTraining(state_dim=15, action_dim=2)
        self.ppo.save_dir = self.save_dir
        self.ppo.save_freq = 1 #10
        self.ppo.min_update_steps = 64

        self.odom = None
        self.scan = None
        self.path = None
        self.goal_xy = None
        self.teacher_cmd = Twist()
        self.prev_cmd = Twist()

        self.prev_goal_dist = None
        self.steps = 0
        self.max_steps = 300 #1500
        
        self.warmup_episodes = 20 # teszt jellegel

        self.timer = self.create_timer(1.0 / hz, self.loop)
        
        """
        self.save_dir = os.path.expanduser("~/ppo_models")
        os.makedirs(self.save_dir, exist_ok=True)
        self.get_logger().info(f"PPO save_dir: {self.save_dir}")
        self.ppo.save_dir = self.save_dir
        """

        self.get_logger().info("PPO training controller elindult (egyszerű verzió).")

    # ---------------- callbacks ----------------
    def odom_cb(self, msg):
        self.odom = msg

    def scan_cb(self, msg):
        self.scan = msg

    def path_cb(self, msg):
        self.path = msg
        if msg.poses:
            gx = msg.poses[-1].pose.position.x
            gy = msg.poses[-1].pose.position.y
            self.goal_xy = (gx, gy)

    def teacher_cb(self, msg):
        self.teacher_cmd = msg

    # ---------------- fő loop ----------------
    def loop(self):
        if self.odom is None or self.scan is None or self.path is None or self.goal_xy is None:
            return

        # 1) state (15 dim): 10 lidar + v + w + goal_dist + path_dist + min_lidar
        state = self.make_state()

        # 2) PPO action (delta v, delta w)
        st = torch.tensor(state, dtype=torch.float32).unsqueeze(0)
        with torch.no_grad():
            dist, _ = self.ppo.old_policy(st)
            a = dist.mean
            log_prob = dist.log_prob(a).sum(-1).item()
            a = a.squeeze(0).cpu().numpy()

        #delta_v = float(a[0]) * self.delta_v_max
        #delta_w = float(a[1]) * self.delta_w_max
        #delta_v = 0.0
        #delta_w = 0.0
        #delta_v = float(a[0]) * 0.02   # nagyon kicsi
        #delta_w = float(a[1]) * 0.10
        
        if self.ppo.episode < self.warmup_episodes:
            delta_v = 0.0
            delta_w = 0.0
        else:
            delta_v = float(a[0]) * 0.02
            delta_w = float(a[1]) * 0.10

        # 3) teacher + delta -> cmd_vel
        cmd = Twist()
        cmd.linear.x = float(self.teacher_cmd.linear.x + delta_v)
        cmd.angular.z = float(self.teacher_cmd.angular.z + delta_w)

        # kicsi clamp, ne menjen el
        cmd.linear.x = float(np.clip(cmd.linear.x, -0.05, 0.35))
        cmd.angular.z = float(np.clip(cmd.angular.z, -1.5, 1.5))

        self.pub_cmd.publish(cmd)

        # 4) reward
        reward, done = self.reward_done(cmd)

        # 5) store
        self.ppo.store(state, np.array([delta_v, delta_w], dtype=np.float32), log_prob, reward, done)

        self.steps += 1
        if self.steps >= self.max_steps:
            done = True

        if done:
            self.ppo.finish_episode()
            self.reset_episode()

    def reset_episode(self):
        self.prev_goal_dist = None
        self.steps = 0

    # ---------------- state ----------------
    def make_state(self):
        # lidar -> 10 szektor min
        ranges = np.array(self.scan.ranges, dtype=np.float32)
        ranges = np.nan_to_num(ranges, nan=self.scan.range_max, posinf=self.scan.range_max, neginf=0.0)
        ranges = np.clip(ranges, 0.0, self.scan.range_max)

        sectors = np.array_split(ranges, 10)
        lidar10 = np.array([np.min(s) for s in sectors], dtype=np.float32)
        lidar10 = np.clip(lidar10 / max(self.scan.range_max, 1e-6), 0.0, 1.0)

        v = float(self.odom.twist.twist.linear.x)
        w = float(self.odom.twist.twist.angular.z)

        px = float(self.odom.pose.pose.position.x)
        py = float(self.odom.pose.pose.position.y)
        gx, gy = self.goal_xy

        goal_dist = float(math.hypot(gx - px, gy - py))

        # path_dist: legközelebbi path ponthoz távolság (nagyon egyszerű)
        path_dist = 0.0
        if self.path.poses:
            pts = [(p.pose.position.x, p.pose.position.y) for p in self.path.poses]
            d2 = [(x - px) ** 2 + (y - py) ** 2 for x, y in pts]
            path_dist = float(math.sqrt(min(d2)))

        min_lidar = float(np.min(ranges))

        # 10 + 5 = 15 dim
        state = np.concatenate([
            lidar10,
            np.array([v, w, goal_dist, path_dist, min_lidar], dtype=np.float32)
        ], axis=0)

        return state

    # ---------------- reward ----------------
    def reward_done(self, cmd):
        # --- alap adatok ---
        ranges = np.array(self.scan.ranges, dtype=np.float32)
        ranges = np.nan_to_num(ranges, nan=self.scan.range_max, posinf=self.scan.range_max, neginf=0.0)
        min_lidar = float(np.min(ranges))

        px = float(self.odom.pose.pose.position.x)
        py = float(self.odom.pose.pose.position.y)
        gx, gy = self.goal_xy
        goal_dist = float(math.hypot(gx - px, gy - py))

        # --- terminál feltételek ---
        if min_lidar < 0.18:
            return -50.0, True

        if goal_dist < 0.2:
            return 100.0, True

        # --- progress reward (közeledés a célhoz) ---
        if self.prev_goal_dist is None:
            self.prev_goal_dist = goal_dist
        progress = self.prev_goal_dist - goal_dist
        self.prev_goal_dist = goal_dist

        # --- path deviation (távolság a path-tól) ---
        path_dist = 0.0
        if self.path is not None and self.path.poses:
            pts = [(p.pose.position.x, p.pose.position.y) for p in self.path.poses]
            d2 = [(x - px) ** 2 + (y - py) ** 2 for x, y in pts]
            path_dist = float(math.sqrt(min(d2)))

        # --- energy / smoothness (sebességváltozás) ---
        dv = abs(cmd.linear.x - self.prev_cmd.linear.x)
        dw = abs(cmd.angular.z - self.prev_cmd.angular.z)
        self.prev_cmd = cmd  # eltesszük a következő lépéshez

        # --- obstacle közelség bünti (hogy ne “tolja rá” a falra) ---
        obstacle_pen = 0.0
        if min_lidar < 0.4:
            obstacle_pen = -1.0

        # --- teacher követés (enyhe bünti) ---
        teacher_pen = -0.05 * (
            abs(cmd.linear.x - self.teacher_cmd.linear.x) +
            abs(cmd.angular.z - self.teacher_cmd.angular.z)
        )

        # --- végső reward (egyszerű súlyok) ---
        r = 10.0 * progress
        r += obstacle_pen
        r += -1.0 * path_dist          # path deviation
        r += -0.5 * (dv + dw)          # energy / rángatás
        r += teacher_pen

        return float(r), False



def main(args=None):
    rclpy.init(args=args)
    node = PPOTrainingController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
