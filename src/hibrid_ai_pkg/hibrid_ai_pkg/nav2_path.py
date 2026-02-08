import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.action import ActionClient

from nav_msgs.msg import Path
from nav2_msgs.action import FollowPath


class Nav2PathClient(Node):
    '''Ez a node kezeli a Path üzeneteket és továbbítja a Nav2 FollowPath action-nek'''
    
    def __init__(self):
        super().__init__('nav2_path_client')

        self.get_logger().info('Nav2 Path Client node indul...')

        # paramétert beolvasássuk
        self.declare_parameter('path_topic', 'planned_path_dilated')
        path_topic = self.get_parameter('path_topic').get_parameter_value().string_value

        # itt határozzuk meg, hogyan a node hogyan fogadja az üzeneteket.
        qos = QoSProfile(depth=10) # buffer mérete, 10 üzenet tárolódik a subscribernek, ha lemaradna
        qos.reliability = ReliabilityPolicy.RELIABLE # ezzel garantáljuk, hogy minden üzenetet biztosan megkapson a subscirber
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL # amikor egy subscriber később csatlakozik megkapja az utlsó üzenetet amit a publisher küldött

        self._path_sub = self.create_subscription(Path, path_topic, self.path_callback, qos)

        # létrehozunk egy action client-et a Nav2 FollowPath-hoz
        self._client = ActionClient(self, FollowPath, 'follow_path')

        # ahogyan logger info is mutatja megvárjuk, hogy az action szerver elérhető legyen
        self.get_logger().info('Várakozás a FollowPath action szerverre...')
        self._client.wait_for_server()
        self.get_logger().info('FollowPath action szerver elérhető.')

        self.get_logger().info('Nav2 Path Client node inicializálva...')

    def path_callback(self, msg: Path):
        # ha nem kapunk üzenetet a topikon keresztül, akkor nem csinálunk semmit
        if not msg.poses:
            self.get_logger().warn('Nincs Path message...')
            return

        self.get_logger().info( f'A Path message megérkezett {len(msg.poses)}, továbbítás a Nav2 felé...')

        # létrehozunk egy FollowPath.Goal objektumot, itt határozzuk meg, hogy a robot mit csináljon
        goal_msg = FollowPath.Goal()
        goal_msg.path = msg # beállítjuk az útvonalat amit a robotnak követni kell

        # aszinkron módon elküldjük az üzenetet, útvonalat az action szervernek, erre a robot elkezd mozogni és várunk egy visszajelzésre, result-ra
        send_goal_future = self._client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)
      
    # amikor megérkezik a visszajelzés, megnézzük, hogy az action szerver elfogadta-e  
    def goal_response_callback(self, future):
        goal_handle = future.result()
        
        if not goal_handle.accepted:
            self.get_logger().warn('FollowPath cél elutasítva...')
            return

        # ha elfogadta elkezdjük várni az eredményt
        self.get_logger().info('FollowPath cél elfogadva, várunk a resultra...')
        result_future = goal_handle.get_result_async()
        # a result_callback automatikusan meghívódik, amikor a Future objektum befejzte a működését éd átadja a Future objektumot 'rf'
        result_future.add_done_callback(self.result_callback) 
        
    # amikor a robot befejezte az útvonal követését, a callback kiírja az eredményt, rf pontosan az a Future objektum, amit a Nav2 visszaküld a robot követési eredményével
    def result_callback(self, rf):
        result = rf.result().result
        self.get_logger().info(f'FollowPath befejezte a működést: {result.result}')


def main(args=None):
    rclpy.init(args=args)
    node = Nav2PathClient()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
