import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


class TfBroadcaster(Node):
    '''az Odometry üzeneteket tf2 transzformációvá alakítása'''
    
    def __init__(self):
        super().__init__('tf_broadcaster')
        odom_topic = self.declare_parameter('odom_topic', 'odom').get_parameter_value().string_value
        
        # létrehozunk egy TransformBroadcaster objektumot, ez teszi lehetővé a tf2 üzenetek küldését
        self.transform_broadcaster = TransformBroadcaster(self)
        
        qos = QoSProfile(depth=10)
        # felirakozunk egy Odometry típusú odom_topic-ra és amikor új üzenet érkezik, akkor az odom_callback meghívódik
        self.subscription = self.create_subscription(Odometry, odom_topic, self.odom_callback, qos)

        self.get_logger().info( f'Odom - TF broadcaster elindult... {odom_topic}')

    # Odometry üzenet átalakítjuk tf2-re, hogy az RViz és a navigáció lássa
    def odom_callback(self, msg):
        # létrehozunk egy TransformStamped objektumot, ami azt mondja meg, hol van egy dolog a térben egy másikhoz képest.
        tfs = TransformStamped()
         # itt állítjuk be, hogy a transzformáció az odom koordinátarendszerből, referenciarendszerből indul, és az adott időpontban érvényes.
        tfs.header.frame_id = 'odom' 
        tfs.header.stamp = msg.header.stamp
        tfs.child_frame_id =  'base_link'

        # itt adjuk meg, hogy hol van child frame  a header_framehez viszonyítva, megadjuk a pozíciót és az irányt
        tfs.transform.translation.x = msg.pose.pose.position.x
        tfs.transform.translation.y = msg.pose.pose.position.y
        tfs.transform.translation.z = msg.pose.pose.position.z
        tfs.transform.rotation = msg.pose.pose.orientation

        # itt küldjük az előzőleg összepakolt tf üzenetet a ROS2 többi node-ja felé
        self.transform_broadcaster.sendTransform(tfs)


def main(args=None):
    rclpy.init(args=args)
    node = TfBroadcaster()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
