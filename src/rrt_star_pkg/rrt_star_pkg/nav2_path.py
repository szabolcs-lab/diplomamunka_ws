import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.action import ActionClient

from nav_msgs.msg import Path
from nav2_msgs.action import FollowPath


class Nav2PathClient(Node):
    def __init__(self):
        super().__init__('nav2_path_client')

        self.get_logger().info('nav2_path_client node initializing...')

        # Paraméter: melyik Path topikra figyeljünk
        self.declare_parameter('path_topic', 'planned_path_dilated')
        path_topic = self.get_parameter('path_topic').get_parameter_value().string_value

        # QoS: ugyanaz a logika, mint a path publishernél
        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self._path_sub = self.create_subscription(
            Path,
            path_topic,
            self.path_callback,
            qos
        )

        # Action client Nav2 FollowPath-hoz
        self._client = ActionClient(self, FollowPath, 'follow_path')

        self.get_logger().info('Waiting for FollowPath action server...')
        self._client.wait_for_server()
        self.get_logger().info('FollowPath action server is available.')

        self.get_logger().info('nav2_path_client node initialized.')

    def path_callback(self, msg: Path):
        if not msg.poses:
            self.get_logger().warn('Received empty path, ignoring.')
            return

        self.get_logger().info(
            f'Received path with {len(msg.poses)} poses, sending to Nav2...'
        )
        
        self.get_logger().info(
            f'Received path with {len(msg.poses)} poses, first=({msg.poses[0].pose.position.x:.2f}, {msg.poses[0].pose.position.y:.2f}), '
            f'last=({msg.poses[-1].pose.position.x:.2f}, {msg.poses[-1].pose.position.y:.2f})'
        )

        goal_msg = FollowPath.Goal()
        goal_msg.path = msg

        send_goal_future = self._client.send_goal_async(goal_msg)

        def goal_response_callback(future):
            goal_handle = future.result()
            if not goal_handle.accepted:
                self.get_logger().warn('FollowPath goal was rejected.')
                return

            self.get_logger().info('FollowPath goal accepted, waiting for result...')
            result_future = goal_handle.get_result_async()

            def result_callback(rf):
                result = rf.result().result
                self.get_logger().info(f'FollowPath finished with result: {result.result}')

            result_future.add_done_callback(result_callback)

        send_goal_future.add_done_callback(goal_response_callback)


def main(args=None):
    rclpy.init(args=args)
    node = Nav2PathClient()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
