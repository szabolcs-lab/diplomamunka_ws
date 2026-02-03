import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import Twist

import numpy as np
import math

from ppo_agent import PPOAgent
from ppo_training import PPOTraining


class PPOController(Node):
    """PPO path following: odom + path → cmd_vel"""

    STATE_DIM = 6  #dist_goal, dist_path, angle_to_next, v, w, prev_v

    def __init__(self):
        super().__init__("ppo_controller")

        self.params = {
            "odom_topic": self.declare_parameter("odom_topic", "/odom").value,
            "path_topic": self.declare_parameter("path_topic", "/planned_path_dilated").value,
            "cmd_vel_topic": self.declare_parameter("cmd_vel_topic", "/cmd_vel").value,
            "hz": float(self.declare_parameter("control_rate_hz", 10.0).value),

            "train": bool(self.declare_parameter("training_mode", True).value),
            "checkpoint": (self.declare_parameter("checkpoint_path", "").value.strip() or None),

            "v_min": float(self.declare_parameter("linear_velocity_min", 0.0).value),
            "v_max": float(self.declare_parameter("linear_velocity_max", 0.3).value),
            "w_max": float(self.declare_parameter("angular_velocity_max", 1.0).value),

            "goal_tol": float(self.declare_parameter("goal_tolerance", 0.3).value),
            "max_steps": int(self.declare_parameter("max_steps_per_episode", 1000).value),

            #reward súlyok
            "r_progress": float(self.declare_parameter("reward_goal_progress", 2.0).value),
            "r_path": float(self.declare_parameter("penalty_path_distance", 1.0).value),
            "r_smooth": float(self.declare_parameter("penalty_cmd_change", 0.05).value),
        }

        #PPO setup
        self.agent = PPOAgent(state_dim=self.STATE_DIM, action_dim=2, checkpoint_path=self.params["checkpoint"], linear_velocity_min=self.params["v_min"],
            linear_velocity_max=self.params["v_max"], angular_velocity_max=self.params["w_max"],)
        
        self.trainer = PPOTraining(state_dim=self.STATE_DIM, action_dim=2)

        qos = rclpy.qos.QoSProfile(
            depth=10,
            reliability=rclpy.qos.ReliabilityPolicy.RELIABLE
        )
        
        self.odom_sub = self.create_subscription(Odometry, self.params["odom_topic"], self._odom_cb, qos)
        self.path_sub = self.create_subscription(Path, self.params["path_topic"], self._path_cb, qos)
        self.cmd_pub = self.create_publisher(Twist, self.params["cmd_vel_topic"], qos)

        #állapot
        self.odom = None
        self.path = None

        self.prev_dist_goal = None         #scalar vagy None
        self.prev_cmd_vel = np.zeros(2, dtype=np.float32)  #[v_cmd, w_cmd]
        self.steps = 0

        #timer
        self.create_timer(1.0 / self.params["hz"], self.control_loop)

        self.get_logger().info(f"PPO controller indul | train={self.params['train']} | v_max={self.params['v_max']}")

    def _odom_cb(self, msg):
        self.odom = msg

    def _path_cb(self, msg):
        if len(msg.poses) > 1:
            self.path = np.array([[p.pose.position.x, p.pose.position.y] for p in msg.poses], dtype=np.float32)

    def control_loop(self):
        if self.odom is None or self.path is None:
            return

        #robot adatai
        robot_x = self.odom.pose.pose.position.x
        robot_y = self.odom.pose.pose.position.y
        v_odom = self.odom.twist.twist.linear.x
        w_odom = self.odom.twist.twist.angular.z

        robot_xy = np.array([robot_x, robot_y], dtype=np.float32)

        #Path: nearest + next
        diffs = self.path - robot_xy
        nearest_idx = int(np.argmin(np.sum(diffs * diffs, axis=1)))
        next_idx = min(nearest_idx + 1, len(self.path) - 1)

        nearest_point = self.path[nearest_idx]
        next_point = self.path[next_idx]
        goal_point = self.path[-1]

        dist_path = float(np.linalg.norm(nearest_point - robot_xy))
        dist_goal = float(np.linalg.norm(goal_point - robot_xy))

        #angle_to_next: atan2(dy, dx)
        dx, dy = (next_point - robot_xy)
        angle_to_next = math.atan2(float(dy), float(dx))

        #State (6D)
        state = np.array([dist_goal,dist_path,angle_to_next,float(v_odom),float(w_odom),float(self.prev_cmd_vel[0]),], dtype=np.float32)

        #action
        deterministic = (not self.params["train"])
        raw_action, cmd_vel, log_prob = self.agent.select_action(state, deterministic=deterministic)

        #publish cmd_vel
        twist = Twist()
        twist.linear.x = float(cmd_vel[0])
        twist.angular.z = float(cmd_vel[1])
        self.cmd_pub.publish(twist)

        #reward
        if self.prev_dist_goal is None:
            progress = 0.0
        else:
            progress = self.prev_dist_goal - dist_goal  # pozitív ha közelebb ment

        dv = float(cmd_vel[0] - self.prev_cmd_vel[0])
        dw = float(cmd_vel[1] - self.prev_cmd_vel[1])
        cmd_change = dv * dv + dw * dw

        reward = (self.params["r_progress"] * progress - self.params["r_path"] * dist_path - self.params["r_smooth"] * cmd_change)

        #goal bonus
        if dist_goal < self.params["goal_tol"]:
            reward = reward + 10.0

        done = (dist_goal < self.params["goal_tol"]) or (self.steps >= self.params["max_steps"])

        #training
        if self.params["train"]:
            self.trainer.store_transition(state, raw_action, log_prob, reward, done)
            if done:
                self.trainer.end_episode()

        #következő stephez
        self.prev_dist_goal = dist_goal
        self.prev_cmd_vel = cmd_vel.copy()
        self.steps = self.steps + 1

        if done:
            self._reset_episode()

    def _reset_episode(self):
        stop = Twist()
        stop.linear.x = 0.0
        stop.angular.z = 0.0
        self.cmd_pub.publish(stop)

        self.prev_dist_goal = None
        self.prev_cmd_vel[:] = 0.0
        self.steps = 0


def main(args=None):
    rclpy.init(args=args)
    node = PPOController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()
