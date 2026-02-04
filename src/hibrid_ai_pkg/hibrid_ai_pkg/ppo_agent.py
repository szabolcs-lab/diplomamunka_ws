#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
import numpy as np
import torch
import math
from collections import deque
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path, Odometry
from sensor_msgs.msg import LaserScan

# Saját modulok
from .actor_critic_network import ActorCriticNetwork
from .ppo_memory import PPOMemory
from .ppo_training import PPOTraining

class PurePPOAgent(Node):
    def __init__(self):
        super().__init__('pure_ppo_agent')
        self.get_logger().info('🧠 PURE PPO 15D - ROBOT@PATH_END')
        
        # PPO 15D
        self.ppo_trainer = PPOTraining(state_dim=15, action_dim=2, save_dir="./ppo_models")
        
        # Állapotok
        self.current_pose = np.array([0.0, 0.0, 0.0])
        self.path_poses = []
        self.raw_dstar_path = []
        self.laser_ranges = []
        self.vel_history = deque(maxlen=5)
        self.last_ang_vel = 0.0
        self.step_count = 0
        
        # GRID PARAMS
        self.grid_resolution = 0.1
        self.grid_size = 200
        self.grid_origin = np.array([-10.0, -10.0])
        
        # QoS
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        
        self.path_sub = self.create_subscription(Path, '/planned_path_dilated', self.path_callback, qos)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, qos)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, qos)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', qos)
        
        self.control_timer = self.create_timer(0.05, self.ppo_loop)
        self.get_logger().info('✅ PATH FOLLOWING READY')

    def grid_to_world(self, grid_x, grid_y):
        world_x = self.grid_origin[0] + grid_x * self.grid_resolution
        world_y = self.grid_origin[1] + grid_y * self.grid_resolution
        return world_x, world_y

    def world_to_grid(self, world_x, world_y):
        grid_x = int((world_x - self.grid_origin[0]) / self.grid_resolution)
        grid_y = int((world_y - self.grid_origin[1]) / self.grid_resolution)
        return grid_x, grid_y

    def path_callback(self, msg):
        """🎯 D* RAW vs WORLD + ROBOT DIAGNOSZTIKA"""
        self.path_poses = []
        self.raw_dstar_path = [(float(pose.pose.position.x), float(pose.pose.position.y)) 
                              for pose in msg.poses]
        
        for pose in msg.poses:
            grid_x = float(pose.pose.position.x)
            grid_y = float(pose.pose.position.y)
            world_x, world_y = self.grid_to_world(grid_x, grid_y)
            self.path_poses.append((world_x, world_y))
        
        # PONTOS DIAGNOSZTIKA
        start_raw = self.raw_dstar_path[0]
        end_raw = self.raw_dstar_path[-1]
        robot_grid = self.world_to_grid(self.current_pose[0], self.current_pose[1])
        
        self.get_logger().info(f"🎯 D* RAW: Start({int(start_raw[0])},{int(start_raw[1])}) → "
                              f"End({int(end_raw[0])},{int(end_raw[1])})")
        self.get_logger().info(f"🤖 WORLD: ({self.current_pose[0]:.1f},{self.current_pose[1]:.1f}) "
                              f"→ GRID({robot_grid[0]},{robot_grid[1]})")

    def odom_callback(self, msg):
        pose = msg.pose.pose
        siny_cosp = 2 * (pose.orientation.w * pose.orientation.z)
        cosy_cosp = 1 - 2 * (pose.orientation.x**2 + pose.orientation.z**2)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        self.current_pose = np.array([float(pose.position.x), float(pose.position.y), yaw])

    def scan_callback(self, msg):
        self.laser_ranges = list(msg.ranges)[:360]

    def find_adaptive_target(self):
        """🔧 ROBOT A PATH VÉGÉNÉL → KERESd A KÖVETKEZŐ PONTOT!"""
        if not self.path_poses:
            return None
        
        pos_x, pos_y = self.current_pose[:2]
        
        # 1. TALÁLD MEG A LEGKÖZELEBBI path pontot
        closest_idx = 0
        min_dist = float('inf')
        
        for i in range(len(self.path_poses)):
            px, py = self.path_poses[i]
            dist = math.hypot(px-pos_x, py-pos_y)
            if dist < min_dist:
                min_dist = dist
                closest_idx = i
        
        # 2. Lookahead: 1-2.5m-re ELŐRE a path-en
        target_idx = closest_idx
        for lookahead in range(closest_idx + 1, min(closest_idx + 20, len(self.path_poses))):
            px, py = self.path_poses[lookahead]
            dist_ahead = math.hypot(px-pos_x, py-pos_y)
            if 1.0 <= dist_ahead <= 2.5:
                target_idx = lookahead
                break
        
        # 3. Ha path végén, akkor visszafelé (kisebb index)
        if closest_idx > len(self.path_poses) * 0.95:
            target_idx = max(0, closest_idx - 8)
        
        return target_idx

    def ppo_loop(self):
        if len(self.path_poses) == 0 or len(self.laser_ranges) < 30:
            self.publish_zero_velocity()
            return
        
        state = self.get_15d_state()
        if state is None:
            return
        
        try:
            action, log_prob = self.ppo_policy(state)
            
            safe_lin = max(0.15, float(np.clip(action[0], 0.15, 0.6)))
            raw_ang = float(np.clip(action[1], -2.5, 2.5))
            smooth_ang = 0.7 * raw_ang + 0.3 * self.last_ang_vel
            self.last_ang_vel = smooth_ang
            
            reward = self.compute_reward(safe_lin, smooth_ang)
            safe_action = np.array([safe_lin, smooth_ang])
            self.ppo_trainer.store_transition(state, safe_action, log_prob, reward, False)
            
            twist = Twist()
            twist.linear.x = safe_lin
            twist.angular.z = smooth_ang
            self.cmd_pub.publish(twist)
            
            self.vel_history.append(safe_lin)
            self.step_count += 1
            
            if self.step_count % 30 == 0:
                target_idx = self.find_adaptive_target()
                if target_idx is not None:
                    tx, ty = self.path_poses[target_idx]
                    dist = math.dist(self.current_pose[:2], self.path_poses[target_idx])
                    robot_grid = self.world_to_grid(self.current_pose[0], self.current_pose[1])
                    target_grid = self.world_to_grid(tx, ty)
                    
                    self.get_logger().info(f"🤖G({robot_grid[0]:2d},{robot_grid[1]:2d}) "
                                         f"[{target_idx:3d}/{len(self.path_poses)}]→G({target_grid[0]:3d},{target_grid[1]:3d}) "
                                         f"d:{dist:.1f} lin:{safe_lin:.2f}")
        except Exception as e:
            self.get_logger().error(f"❌ Error: {e}")
            self.publish_zero_velocity()

    def get_15d_state(self):
        try:
            pos_x, pos_y, yaw = self.current_pose
            
            target_idx = self.find_adaptive_target()
            if target_idx is None:
                return None
            
            tx, ty = self.path_poses[target_idx]
            rel_x = tx - pos_x
            rel_y = ty - pos_y
            dist = max(math.sqrt(rel_x**2 + rel_y**2), 0.1)
            angle = math.atan2(rel_y, rel_x) - yaw
            angle = np.clip((angle + math.pi) % (2*math.pi) - math.pi, -math.pi, math.pi)
            
            progress = target_idx / max(len(self.path_poses), 1)
            
            if self.laser_ranges:
                front_slice = slice(max(0, 315), min(360, 45))
                front_ranges = [r for r in self.laser_ranges[front_slice] if 0 < r < 20]
                laser_front = np.mean(front_ranges) if front_ranges else 5.0
                
                valid_ranges = [r for r in self.laser_ranges if 0 < r < 20]
                laser_min = min(valid_ranges) if valid_ranges else 5.0
                laser_avg = np.mean(valid_ranges) if valid_ranges else 5.0
            else:
                laser_front = laser_min = laser_avg = 5.0
            
            avg_vel = np.mean(self.vel_history) if self.vel_history else 0.0
            
            state = np.array([
                np.clip(pos_x/20.0, -1.0, 1.0),
                np.clip(pos_y/20.0, -1.0, 1.0),
                np.clip(yaw/math.pi, -1.0, 1.0),
                np.clip(rel_x/dist, -1.0, 1.0),
                np.clip(rel_y/dist, -1.0, 1.0),
                np.clip(angle/math.pi, -1.0, 1.0),
                np.clip(laser_front/10.0, 0.0, 2.0),
                np.clip(laser_min/10.0, 0.0, 2.0),
                np.clip(laser_avg/10.0, 0.0, 2.0),
                progress,
                np.clip(dist/3.0, 0.0, 1.0),
                np.clip(avg_vel/0.6, 0.0, 1.0),
                np.clip(1.0/(laser_min+0.1)/10.0, 0.0, 1.0),
                0.0, 0.0
            ], dtype=np.float32)
            
            return state if not (np.any(np.isnan(state)) or np.any(np.isinf(state))) else None
        except:
            return None

    def ppo_policy(self, state):
        state_tensor = torch.tensor(state, dtype=torch.float32).unsqueeze(0)
        distribution, _ = self.ppo_trainer.actor_critic(state_tensor)
        action = distribution.sample().cpu().numpy()[0]
        log_prob = distribution.log_prob(torch.tensor(action, dtype=torch.float32).unsqueeze(0)).sum()
        return action, log_prob.item()

    def compute_reward(self, lin_vel, ang_vel):
        reward = 0.0
        
        target_idx = self.find_adaptive_target()
        if target_idx is not None and target_idx < len(self.path_poses):
            dist = math.dist(self.current_pose[:2], self.path_poses[target_idx])
            reward += 50.0 * (1.0 - min(dist/1.2, 1.0))
        
        reward += 20.0 * lin_vel
        
        if self.laser_ranges:
            min_dist = min([r for r in self.laser_ranges if r > 0] or [10.0])
            if min_dist < 0.4:
                reward -= 350.0
            elif min_dist < 0.7:
                reward -= 70.0
        
        if self.vel_history:
            vel_change = abs(lin_vel - self.vel_history[-1])
            reward -= 12.0 * vel_change
        
        return np.clip(reward, -400, 100)

    def publish_zero_velocity(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.cmd_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = PurePPOAgent()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('🛑 PPO stopped')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
