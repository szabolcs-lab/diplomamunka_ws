import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


class TfBroadcaster(Node):
    def __init__(self):
        super().__init__('tf_broadcaster')

        qos = QoSProfile(depth=10)
        odom_topic = self.declare_parameter('odom_topic', 'odom').get_parameter_value().string_value
        self._tf_br = TransformBroadcaster(self)

        self._sub = self.create_subscription(Odometry, odom_topic, self.odom_callback, qos)

        self.get_logger().info( f'Odom TF broadcaster started, listening on {odom_topic}')

    def odom_callback(self, msg: Odometry):
        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = 'odom'
        t.child_frame_id =  'base_link'

        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        t.transform.rotation = msg.pose.pose.orientation

        self._tf_br.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = TfBroadcaster()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
