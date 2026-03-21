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

        self.declare_parameter('path_topic', '/planned_path_dilated')
        path_topic = self.get_parameter('path_topic').get_parameter_value().string_value
        
        self.declare_parameter("minimum_preemption_time", 2.0)      # sec
        self.minimum_preemption_time = float(self.get_parameter("minimum_preemption_time").value)
        
        self.declare_parameter("maximum_goal_shift_distance", 0.30)  # m
        self.maximum_goal_shift_distance = float(self.get_parameter("maximum_goal_shift_distance").value)
        
        self.declare_parameter("goal_reached_tolerance", 0.8)  # m  
        self.goal_reached_tolerance = float(self.get_parameter("goal_reached_tolerance").value)
        
        self.declare_parameter("path_change_thresh", 0.25)  # m
        self.path_change_thresh = float(self.get_parameter("path_change_thresh").value)

        self.declare_parameter("path_change_check_points", 25)  # db pose
        self.path_change_check_points = int(self.get_parameter("path_change_check_points").value)

        self.last_goal_sent_time = 0.0    
        self.last_goal_xy = None 
        self.goal_active = False
        self.goal_handle = None
        self.pending_path = None
        self.cancel_in_progress = False
        self.last_front_path = None

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        qos = QoSProfile(depth=10) 
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL 

        self._path_sub = self.create_subscription(Path, path_topic, self.path_callback, qos)

        # létrehozunk egy action client-et a Nav2 FollowPath-hoz
        self._client = ActionClient(self, FollowPath, 'follow_path')

        self.get_logger().info('Várakozás a FollowPath action szerverre...')
        self._client.wait_for_server()
        
        self.get_logger().info('FollowPath action szerver elérhető...')
        self.get_logger().info('Nav2 Path Client node inicializálva...')

    # Path callback - ide érkezik minden új Path
    def path_callback(self, msg: Path):

        if not msg.poses:
            self.get_logger().warn('Nincs Path message...')
            return

        msg2 = self.slice_path_to_robot(msg)
        
        if self.robot_close_to_path_goal(msg2):
            self.get_logger().info("Robot már cél közelében van, új Path ignorálva...")
            return

        if len(msg2.poses) < 3:
            self.get_logger().warn('Túl rövid a path avágás után, nem küldöm FollowPath-nek...')
            return

        if self.goal_active and self.goal_handle is not None:

            if self.cancel_in_progress:
                return

            if not self.is_preempt(msg2):
                return

            self.pending_path = msg2

            self.cancel_in_progress = True
            self.get_logger().info('Új Path jött goal alatt - cancel régi FollowPath...')

            cancel_future = self.goal_handle.cancel_goal_async()
            cancel_future.add_done_callback(self.cancel_done_callback)
            return

        self.get_logger().info(f'A Path message megérkezett {len(msg.poses)}, küldés a Nav2nek; vágás után {len(msg2.poses)}...')
        self.send_path_as_goal(msg2)


    # Cancel befejeződött callback
    def cancel_done_callback(self, future):

        self.cancel_in_progress = False
        self.goal_active = False
        self.goal_handle = None

        if self.pending_path is not None:
            p = self.pending_path
            self.pending_path = None
            self.get_logger().info('Cancel kész van küldöm az új replanned path-ot...')
            self.send_path_as_goal(p)
            
            
    # Goal küldése a Nav2 FollowPath action szervernek...
    def send_path_as_goal(self, msg: Path):
        self.get_logger().info(f'FollowPath goal küldése, poses={len(msg.poses)}...')
        
    
        goal_msg = FollowPath.Goal()
        goal_msg.path = msg

        self.goal_active = True
        self.last_goal_sent_time = time.time()
        self.last_goal_xy = self.path_goal_xy(msg)

        self.last_front_path = self.calculate_front_path(msg)

        send_goal_future = self._client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)

    #Goal response callback
    def goal_response_callback(self, future):

        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().warn('FollowPath cél elutasítva...')
            self.goal_active = False
            self.goal_handle = None
            return

        self.goal_handle = goal_handle
        self.get_logger().info('FollowPath cél elfogadva, várunk a resultra...')

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)
        

    # Result callback - amikor a robot befejezte a követést
    def result_callback(self, rf):
        result = rf.result().result
        self.get_logger().info('FollowPath befejezte a működést...')

        self.goal_active = False
        self.goal_handle = None
        
        
    # A Path elejének levágása a robothoz legközelebbi pontra...
    def slice_path_to_robot(self, path_msg: Path): 
        #MAX_SLICE_PATH = 40
          
        robott_x, robot_y = self.get_actual_robot_pose_in_map()
        
        if robott_x is None:
            return path_msg
        
        closest_path_index = 0
        closest_distance_sq = float('inf')
        
        for i, path_pose in enumerate(path_msg.poses):
            path_x_point = path_pose.pose.position.x
            path_y_point = path_pose.pose.position.y
 
            delta_x = path_x_point - robott_x
            delta_y = path_y_point - robot_y
            
            distance_squared = delta_x * delta_x + delta_y * delta_y
            
            if distance_squared < closest_distance_sq:
                closest_distance_sq = distance_squared
                closest_path_index = i
                     
        #closest_path_index = min(closest_path_index, MAX_SLICE_PATH)
        
        if len(path_msg.poses) - closest_path_index < 3:
            self.get_logger().warn(f"Túl rövid a path ({len(path_msg.poses)-closest_path_index} pont....)")
            return path_msg
        
        sliced_path = Path()
        sliced_path.header = path_msg.header
        sliced_path.poses = path_msg.poses[closest_path_index:]
          
        
        self.get_logger().debug(f"Path levágva: {closest_path_index} - {len(sliced_path.poses)} pont...")
        
        return sliced_path
    
    
    # A robot aktuális pozíciójának lekérdezése map frame-ben...
    def get_actual_robot_pose_in_map(self):
        try:
            tf = self._tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            robot_map_x = tf.transform.translation.x
            robot_map_y = tf.transform.translation.y
            return robot_map_x, robot_map_y

        except TransformException as e:
            self.get_logger().error(f"TF hiba map-base_link!!! : {e}")
            return None, None
    
    
    #Robot elég közel van-e a path végéhez...    
    def robot_close_to_path_goal(self, path_msg: Path):
        ronot_x, robot_y = self.get_actual_robot_pose_in_map()
        
        if ronot_x is None or not path_msg.poses:
            return False

        path_end_x = float(path_msg.poses[-1].pose.position.x)
        path_end_y = float(path_msg.poses[-1].pose.position.y)
        
        euclides_diatnace = math.hypot(path_end_x - ronot_x, path_end_y - robot_y)
        
        return euclides_diatnace < self.goal_reached_tolerance
    
    
    #Eldönti, hogy érdemes-e most preemptelni ...
    def is_preempt(self, new_path: Path):
        now = time.time()

        too_early = now - self.last_goal_sent_time
        if too_early < self.minimum_preemption_time:
            return False

        #ha a célpont nagyon elmozdult, akkor biztos preempt
        new_xy_goal = self.path_goal_xy(new_path)
        if new_xy_goal is None:
            return False

        if self.last_goal_xy is None:
            return True

        delta_x = new_xy_goal[0] - self.last_goal_xy[0]
        delta_y = new_xy_goal[1] - self.last_goal_xy[1]
        goal_distance = math.hypot(delta_x, delta_y)

        if goal_distance > self.maximum_goal_shift_distance:
            return True

        #ha a cél nem mozdult, de az út alakja igen, akkor is preempt
        return self.is_path_changed(new_path)
    
    
    #Path vépontjának x és y koordinátái map frameben  
    def path_goal_xy(self, path_msg: Path):
        if not path_msg.poses:
            return None
        
        map_goal = path_msg.poses[-1].pose.position
        
        return map_goal.x, map_goal.y
    
    
    
    #Ellenőrzi, hogy az új út elég különböző-e a korábbitól.
    def is_path_changed(self, new_path_message: Path):
        new_front_path = self.calculate_front_path(new_path_message)
        
        if new_front_path is None:
            return False
        
        if self.last_front_path is None:
            return True
        
        num_comparison_points = min(len(new_front_path), len(self.last_front_path))
        if num_comparison_points == 0:
            return False
        
        total_distance = 0.0
        for i in range(num_comparison_points):
            previous_x, previous_y = self.last_front_path[i]
            new_x, new_y = new_front_path[i]
            distance = math.hypot(new_x - previous_x, new_y - previous_y)
            total_distance = total_distance + distance
        
        average_deviation = total_distance / num_comparison_points
        
        is_bigger = average_deviation > self.path_change_thresh
        
        return is_bigger
    
    
    #Az út első N pontját beszünl lenyomatot és beletesszük egy  listába....
    def calculate_front_path(self, path_message: Path):
        num_checkpoints = min(len(path_message.poses), self.path_change_check_points)
        
        if num_checkpoints <= 0:
            return None
        
        front_path = []
        
        for i in range(num_checkpoints):
            point = path_message.poses[i].pose.position
            front_path.append((float(point.x), float(point.y)))
        
        return front_path
        

def main(args=None):
    rclpy.init(args=args)
    node = Nav2PathClient()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()