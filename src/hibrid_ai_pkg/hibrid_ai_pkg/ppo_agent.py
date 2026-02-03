import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

from ppo_agent import PPOAgent
from ppo_training import PPOTraining

import math
import numpy as np

class PPOController(Node):
    STATE_DIM = 6  # [dist_goal, dist_path, angle_next, v_odom, w_odom, prev_v_cmd]
    
    ODOM_TOPIC = "/odom"
    PATH_TOPIC = "/planned_path_dilated"
    SCAN_TOPIC = "/scan"
    CMD_VEL_TOPIC = "/cmd_vel"

    CONTROL_HZ = 10.0
    TRAINING_MODE = True          # False = "deploy" (deterministic)

    # robot sebesség korlát
    V_MIN = 0.0
    V_MAX = 0.3
    W_MAX = 1.0

    # epizód vége
    GOAL_TOL = 0.3
    MAX_STEPS = 1000

    # lidar ütközés
    COLLISION_DIST = 0.18
    COLLISION_PENALTY = 20.0

    # reward súlyok
    R_PROGRESS = 2.0
    R_PATH = 1.0
    R_SMOOTH = 0.05
    GOAL_BONUS = 10.0

    # model mentés (ha training)
    SAVE_DIR = "./ppo_models"
    SAVE_FREQ = 50

    def __init__(self):
        super().__init__("ppo_controller")

        # ------------- PPO
        self.agent = PPOAgent(
            state_dim=self.STATE_DIM,
            action_dim=2,
            checkpoint_path=None,      # ha van mentésem, idekerül
            linear_velocity_min=self.V_MIN,
            linear_velocity_max=self.V_MAX,
            angular_velocity_max=self.W_MAX,
        )

        self.trainer = PPOTraining(
            state_dim=self.STATE_DIM,
            action_dim=2,
            save_dir=self.SAVE_DIR,
            save_freq=self.SAVE_FREQ,
        )

        # ------------- QoS
        # PATH: legyen "latch" jellegű (új node megkapja a legutolsó path-ot)
        path_qos = QoSProfile(depth=1)
        path_qos.reliability = ReliabilityPolicy.RELIABLE
        path_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        # ODOM + SCAN: stream, best effort
        stream_qos = QoSProfile(depth=10)
        stream_qos.reliability = ReliabilityPolicy.BEST_EFFORT
        stream_qos.durability = DurabilityPolicy.VOLATILE

        # CMD: parancs
        cmd_qos = QoSProfile(depth=1)
        cmd_qos.reliability = ReliabilityPolicy.BEST_EFFORT
        cmd_qos.durability = DurabilityPolicy.VOLATILE

        # ------------- ROS I/O
        self.create_subscription(Odometry, self.ODOM_TOPIC, self.on_odom, stream_qos)
        self.create_subscription(Path, self.PATH_TOPIC, self.on_path, path_qos)
        self.create_subscription(LaserScan, self.SCAN_TOPIC, self.on_scan, stream_qos)
        self.cmd_pub = self.create_publisher(Twist, self.CMD_VEL_TOPIC, cmd_qos)

        # ------------- Állapot
        self.odom = None
        self.path_xy = None
        self.scan_min = None

        self.prev_goal_dist = None
        self.prev_cmd = np.array([0.0, 0.0], dtype=np.float32)

        self.steps = 0
        self.collisions_total = 0

        self.create_timer(1.0 / self.CONTROL_HZ, self.control_loop)
        self.get_logger().info(f"PPO controller ready | TRAINING_MODE={self.TRAINING_MODE}")

    # ------------------ callbacks
    def on_odom(self, msg: Odometry):
        self.odom = msg

    def on_path(self, msg: Path):
        if len(msg.poses) < 2:
            return
        self.path_xy = np.array([(p.pose.position.x, p.pose.position.y) for p in msg.poses], dtype=np.float32)

    def on_scan(self, msg: LaserScan):
        ranges = np.array(msg.ranges, dtype=np.float32)
        finite = np.isfinite(ranges)
        self.scan_min = float(np.min(ranges[finite])) if np.any(finite) else None

    # ------------------ main loop
    def control_loop(self):
        if self.odom is None or self.path_xy is None:
            return

        # 1) ütközés lidar alapján
        collided = (self.scan_min is not None) and (self.scan_min < self.COLLISION_DIST)

        # 2) robot hely + sebesség odomból
        robot_xy = np.array([float(self.odom.pose.pose.position.x), float(self.odom.pose.pose.position.y)], dtype=np.float32)

        v_odom = float(self.odom.twist.twist.linear.x)
        w_odom = float(self.odom.twist.twist.angular.z)

        # 3) legközelebbi path pont + következő
        diffs = self.path_xy - robot_xy
        nearest = int(np.argmin(np.sum(diffs * diffs, axis=1)))
        nxt = min(nearest + 1, len(self.path_xy) - 1)

        dist_path = float(np.linalg.norm(self.path_xy[nearest] - robot_xy))
        dist_goal = float(np.linalg.norm(self.path_xy[-1] - robot_xy))

        dx, dy = (self.path_xy[nxt] - robot_xy)
        angle_next = math.atan2(float(dy), float(dx))

        # 4) state (6)
        state = np.array([dist_goal, dist_path, angle_next, v_odom, w_odom, float(self.prev_cmd[0])], dtype=np.float32)

        # 5) PPO akció
        deterministic = (not self.TRAINING_MODE)
        raw_action, cmd_vel, logp = self.agent.select_action(state, deterministic=deterministic)

        # 6) cmd_vel publish
        twist = Twist()
        twist.linear.x = float(cmd_vel[0])
        twist.angular.z = float(cmd_vel[1])
        self.cmd_pub.publish(twist)

        # 7) reward
        progress = 0.0 if self.prev_goal_dist is None else (self.prev_goal_dist - dist_goal)
        smooth = float(np.sum((cmd_vel - self.prev_cmd) ** 2))

        reward = (self.R_PROGRESS * progress - self.R_PATH * dist_path - self.R_SMOOTH * smooth)

        if dist_goal < self.GOAL_TOL:
            reward += self.GOAL_BONUS
        if collided:
            reward -= self.COLLISION_PENALTY

        # 8) done
        done = (dist_goal < self.GOAL_TOL) or collided or (self.steps >= self.MAX_STEPS)

        # 9) tanítás (RAW action megy!)
        if self.TRAINING_MODE:
            self.trainer.store_transition(state, raw_action, logp, reward, done)
            if done:
                self.trainer.end_episode()

        # 10) update + reset
        self.prev_goal_dist = dist_goal
        self.prev_cmd = cmd_vel.copy()
        self.steps += 1

        if collided:
            self.collisions_total += 1

        if done:
            self.cmd_pub.publish(Twist())  # stop
            self.get_logger().info(f"EP END | steps={self.steps} | collisions(total)={self.collisions_total}")
            self.prev_goal_dist = None
            self.prev_cmd[:] = 0.0
            self.steps = 0


def main(args=None):
    rclpy.init(args=args)
    node = PPOController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
