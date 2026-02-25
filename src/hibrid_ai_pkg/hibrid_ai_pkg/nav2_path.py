import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.action import ActionClient

from nav_msgs.msg import Path
from nav2_msgs.action import FollowPath

from tf2_ros import Buffer, TransformListener, TransformException

import math
import time


class Nav2PathClient(Node):
    '''Ez a node kezeli a Path üzeneteket és továbbítja a Nav2 FollowPath action-nek'''
    
    def __init__(self):
        super().__init__('nav2_path_client')

        self.get_logger().info('Nav2 Path Client node indul...')

        # paramétert beolvasássuk
        self.declare_parameter('path_topic', '/planned_path_dilated')
        path_topic = self.get_parameter('path_topic').get_parameter_value().string_value
        
        # Preempt szűrők
        self.declare_parameter("min_preempt_dt", 2.0)      # sec
        self.declare_parameter("goal_shift_thresh", 0.30)  # m
        self.declare_parameter("goal_reached_tolerance", 0.8)  # m
        

        self.min_preempt_dt = float(self.get_parameter("min_preempt_dt").value)
        self.goal_shift_thresh = float(self.get_parameter("goal_shift_thresh").value)
        self.goal_reached_tolerance = float(self.get_parameter("goal_reached_tolerance").value)

        self._last_goal_sent_time = 0.0
        
        self._last_goal_xy = None  # (x,y) a legutóbb elküldött path végpontja

        # Ez mutatja, hogy fut-e éppen FollowPath goal
        self._goal_active = False
        
        # Itt tároljuk az aktuális goal handle-t, hogy cancel-elni tudjuk
        self._goal_handle = None

        # Ha goal alatt jön új Path (pl. replannelés után),
        # akkor ide tesszük el ideiglenesen
        self._pending_path = None
        
        # Ez jelzi, hogy éppen folyamatban van-e a cancel
        self._cancel_in_progress = False

        # A Path map frame-ben van, a robot pedig base_link frame-ben.
        # Ezért szükségünk van TF-re, hogy lekérjük a robot aktuális
        # pozícióját map koordinátarendszerben (map -> base_link).
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        # itt határozzuk meg, hogyan a node hogyan fogadja az üzeneteket.
        qos = QoSProfile(depth=10)  # buffer mérete, 10 üzenet tárolódik a subscribernek, ha lemaradna
        qos.reliability = ReliabilityPolicy.RELIABLE  # ezzel garantáljuk, hogy minden üzenetet biztosan megkapson a subscriber
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL  # amikor egy subscriber később csatlakozik megkapja az utolsó üzenetet amit a publisher küldött

        # feliratkozunk a Path topikra
        self._path_sub = self.create_subscription(Path, path_topic, self.path_callback, qos)

        # létrehozunk egy action client-et a Nav2 FollowPath-hoz
        self._client = ActionClient(self, FollowPath, 'follow_path')

        # ahogyan logger info is mutatja megvárjuk, hogy az action szerver elérhető legyen
        self.get_logger().info('Várakozás a FollowPath action szerverre...')
        self._client.wait_for_server()
        self.get_logger().info('FollowPath action szerver elérhető.')

        self.get_logger().info('Nav2 Path Client node inicializálva...')


    # A robot aktuális pozíciójának lekérdezése map frame-ben
    def get_robot_xy_in_map(self):
        try:
            tf = self._tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            x = tf.transform.translation.x
            y = tf.transform.translation.y
            return float(x), float(y)

        except TransformException as e:
            self.get_logger().warn(f"TF hiba (map-base_link): {e}")
            return None, None
        
    def robot_close_to_path_goal(self, path_msg: Path):
        rx, ry = self.get_robot_xy_in_map()
        
        if rx is None or not path_msg.poses:
            return False

        gx = float(path_msg.poses[-1].pose.position.x)
        gy = float(path_msg.poses[-1].pose.position.y)
        
        return math.hypot(gx - rx, gy - ry) < self.goal_reached_tolerance


    # A Path elejének levágása a robothoz legközelebbi pontra
    # Így nem kell "visszatalálni" az út elejére replannelés után
    def slice_path_to_robot(self, path_msg: Path):
        
        rx, ry = self.get_robot_xy_in_map()
        
        # Ha nincs TF adat, akkor nem vágunk, visszaadjuk az eredetit
        if rx is None:
            return path_msg

        best_i = 0
        best_d2 = float('inf')

        # végigmegyünk a Path pontjain és megkeressük a legközelebbit
        for i, ps in enumerate(path_msg.poses):
            dx = ps.pose.position.x - rx
            dy = ps.pose.position.y - ry
            d2 = dx * dx + dy * dy
            
            if d2 < best_d2:
                best_d2 = d2
                best_i = i

        # ha túl kevés pont maradna, akkor inkább nem vágjuk
        if len(path_msg.poses) - best_i < 3:
            return path_msg

        # új Path objektum létrehozása a levágott pontokkal
        out = Path()
        out.header = path_msg.header
        out.poses = path_msg.poses[best_i:]

        return out
    
    
    
    def path_goal_xy(self, path_msg: Path):
        if not path_msg.poses:
            return None
        g = path_msg.poses[-1].pose.position
        return float(g.x), float(g.y)


    def should_preempt(self, new_path: Path) -> bool:
        """Eldönti, hogy érdemes-e most preemptelni (idő + goal változás)."""
        now = time.time()

        #túl hamar? - ne
        if (now - self._last_goal_sent_time) < self.min_preempt_dt:
            return False

        #goal elmozdult-e?
        new_goal = self.path_goal_xy(new_path)
        if new_goal is None:
            return False

        if self._last_goal_xy is None:
            # még nem küldtünk semmit  igen
            return True

        dx = new_goal[0] - self._last_goal_xy[0]
        dy = new_goal[1] - self._last_goal_xy[1]
        dist = math.hypot(dx, dy)

        return dist > self.goal_shift_thresh
       

    # Goal küldése a Nav2 FollowPath action szervernek
    def send_path_as_goal(self, msg: Path):
        self.get_logger().info(f'FollowPath goal küldése, poses={len(msg.poses)}')

        goal_msg = FollowPath.Goal()
        goal_msg.path = msg

        # Goal küldés előtt jelöljük, hogy aktív goal fut
        self._goal_active = True

        # PREEMPT szűrőhöz eltesszük:
        self._last_goal_sent_time = time.time()
        self._last_goal_xy = self.path_goal_xy(msg)

        send_goal_future = self._client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)

    # Path callback - ide érkezik minden új Path
    def path_callback(self, msg: Path):

        if not msg.poses:
            self.get_logger().warn('Nincs Path message...')
            return

        msg2 = self.slice_path_to_robot(msg)
        
        # Ha már a célban vagyunk, ne küldjünk új goal-t (planner még publikál)
        if self.robot_close_to_path_goal(msg2):
            self.get_logger().info("Robot már cél közelében, új Path ignorálva.")
            return

        if len(msg2.poses) < 3:
            self.get_logger().warn('Túl rövid path (vágás után), nem küldöm FollowPath-nek.')
            return

        # Ha fut goal:
        if self._goal_active and self._goal_handle is not None:

            # ha épp cancel folyamatban, ne csináljunk semmit
            if self._cancel_in_progress:
                return

            # ÚJ: csak akkor preempteljünk, ha tényleg érdemes
            if not self.should_preempt(msg2):
                return

            # eltároljuk az új path-ot
            self._pending_path = msg2

            self._cancel_in_progress = True
            self.get_logger().info('Új Path jött goal alatt - cancel régi FollowPath...')

            cancel_future = self._goal_handle.cancel_goal_async()
            cancel_future.add_done_callback(self.cancel_done_callback)
            return

        # Ha nincs aktív goal, azonnal küldjük
        self.get_logger().info(f'A Path message megérkezett {len(msg.poses)}, küldés Nav2 felé (vágás után {len(msg2.poses)})')
        self.send_path_as_goal(msg2)

    # Cancel befejeződött callback
    def cancel_done_callback(self, future):

        self._cancel_in_progress = False
        self._goal_active = False
        self._goal_handle = None

        # Ha volt eltárolt új path, azt most elküldjük
        if self._pending_path is not None:
            p = self._pending_path
            self._pending_path = None
            self.get_logger().info('Cancel kész - küldöm az új replanned path-ot.')
            self.send_path_as_goal(p)

    # Goal response callback
    def goal_response_callback(self, future):

        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().warn('FollowPath cél elutasítva...')
            self._goal_active = False
            self._goal_handle = None
            return

        self._goal_handle = goal_handle
        self.get_logger().info('FollowPath cél elfogadva, várunk a resultra...')

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    # Result callback - amikor a robot befejezte a követést
    def result_callback(self, rf):
        _ = rf.result().result
        self.get_logger().info('FollowPath befejezte a működést.')

        self._goal_active = False
        self._goal_handle = None
        

def main(args=None):
    rclpy.init(args=args)
    node = Nav2PathClient()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()