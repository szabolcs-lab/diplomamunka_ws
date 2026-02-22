import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.action import ActionClient

from nav_msgs.msg import Path
from nav2_msgs.action import FollowPath

from tf2_ros import Buffer, TransformListener, TransformException


class Nav2PathClient(Node):
    """
    Ez a node:
      - feliratkozik egy tervezett Path topikra (pl. /planned_path_dilated)
      - ezt továbbküldi a Nav2 FollowPath action-nek
      - ha közben új Path jön (dinamikus replannelés), akkor cancel + új goal (preempt)

    Fő cél: dinamikus akadálynál se menjen falnak.
    """

    def __init__(self):
        super().__init__("nav2_path_client")
        self.get_logger().info("Nav2 Path Client node indul...")

        # -------------------------
        # PARAMÉTEREK (egyszerűen)
        # -------------------------
        self.declare_parameter("path_topic", "planned_path_dilated")

        # Ne flippelgessünk (ne cancel + új goal túl gyorsan egymás után)
        self.declare_parameter("min_preempt_dt", 0.7)  # sec

        # A cél közelében ne küldjünk új goal-t (planner még publikálhat)
        self.declare_parameter("goal_reached_tolerance", 0.8)  # m

        # Dinamikus replannelésnél gyakran a cél ugyanaz, de az út eleje változik
        self.declare_parameter("path_change_thresh", 0.25)  # m (átlagos eltérés)
        self.declare_parameter("compare_poses_n", 15)       # első N pontot nézzük

        # KRITIKUS: slice csak akkor, ha tényleg közel van a legközelebbi pont.
        # Ez védi a "fal túloldalán geometriailag közel" hibát.
        self.declare_parameter("max_slice_dist", 0.8)  # m

        self.path_topic = self.get_parameter("path_topic").value
        self.min_preempt_dt = float(self.get_parameter("min_preempt_dt").value)
        self.goal_reached_tolerance = float(self.get_parameter("goal_reached_tolerance").value)
        self.path_change_thresh = float(self.get_parameter("path_change_thresh").value)
        self.compare_poses_n = int(self.get_parameter("compare_poses_n").value)
        self.max_slice_dist = float(self.get_parameter("max_slice_dist").value)

        # -------------------------
        # BELSŐ ÁLLAPOT
        # -------------------------
        self._last_goal_sent_time = 0.0                # mikor küldtünk utoljára goal-t (ROS time sec)
        self._last_prefix_pts = None                   # az utoljára elküldött path eleje (list of (x,y))

        self._goal_active = False                      # fut-e FollowPath goal
        self._goal_handle = None                       # hogy tudjunk cancel-elni
        self._pending_path = None                      # ha cancel alatt jön új path, ide tesszük
        self._cancel_in_progress = False               # épp cancel folyamatban van-e

        # -------------------------
        # TF: hogy tudjuk a robot pozícióját a path frame-jében
        # -------------------------
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        # -------------------------
        # SUBSCRIBER a Path-ra
        # -------------------------
        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self._path_sub = self.create_subscription(Path, self.path_topic, self.path_callback, qos)

        # -------------------------
        # ACTION CLIENT a FollowPath-hoz
        # -------------------------
        self._client = ActionClient(self, FollowPath, "follow_path")
        self.get_logger().info("Várakozás a FollowPath action szerverre...")
        self._client.wait_for_server()
        self.get_logger().info("FollowPath action szerver elérhető.")
        self.get_logger().info("Nav2 Path Client inicializálva.")

    # ---------------------------------
    # IDŐKEZELÉS (sim_time kompatibilis)
    # ---------------------------------
    def now_sec(self) -> float:
        """ROS time másodpercben (Gazebo /clock esetén is)."""
        return self.get_clock().now().nanoseconds * 1e-9

    # ---------------------------------
    # ROBOT POZÍCIÓ: mindig a Path frame-jében!
    # ---------------------------------
    def get_robot_xy_in_frame(self, frame_id: str):
        """
        Robot (base_link) pozíciója a megadott frame-ben.
        A Path.header.frame_id tipikusan 'map' (vagy amit használsz).
        """
        if not frame_id:
            frame_id = "map"

        try:
            tf = self._tf_buffer.lookup_transform(frame_id, "base_link", rclpy.time.Time())
            x = tf.transform.translation.x
            y = tf.transform.translation.y
            return float(x), float(y)
        except TransformException as e:
            self.get_logger().warn(f"TF hiba ({frame_id} -> base_link): {e}")
            return None, None

    # ---------------------------------
    # Segéd: path célpont (x,y)
    # ---------------------------------
    def path_goal_xy(self, path_msg: Path):
        if not path_msg.poses:
            return None
        p = path_msg.poses[-1].pose.position
        return float(p.x), float(p.y)

    # ---------------------------------
    # Segéd: robot közel van-e a path célhoz
    # ---------------------------------
    def robot_close_to_path_goal(self, path_msg: Path) -> bool:
        if not path_msg.poses:
            return False

        frame = path_msg.header.frame_id
        rx, ry = self.get_robot_xy_in_frame(frame)
        if rx is None or ry is None:
            return False

        gx, gy = self.path_goal_xy(path_msg)
        if gx is None:
            return False

        return math.hypot(gx - rx, gy - ry) < self.goal_reached_tolerance

    # ---------------------------------
    # Path eleje (első N pont) listába
    # ---------------------------------
    def path_prefix_xy(self, path_msg: Path, n: int):
        pts = []
        for ps in path_msg.poses[:max(1, n)]:
            pts.append((float(ps.pose.position.x), float(ps.pose.position.y)))
        return pts

    # ---------------------------------
    # Dinamikus replannelés detektálása:
    # a path eleje változott-e sokat?
    # ---------------------------------
    def path_prefix_changed(self, new_path: Path) -> bool:
        if self._last_prefix_pts is None:
            return True

        new_pts = self.path_prefix_xy(new_path, self.compare_poses_n)
        old_pts = self._last_prefix_pts

        m = min(len(new_pts), len(old_pts))
        if m < 3:
            return True

        s = 0.0
        for i in range(m):
            dx = new_pts[i][0] - old_pts[i][0]
            dy = new_pts[i][1] - old_pts[i][1]
            s += math.hypot(dx, dy)

        mean = s / m
        return mean > self.path_change_thresh

    # ---------------------------------
    # KRITIKUS: Path levágása a robothoz közelebbi részre
    # De csak akkor vágunk, ha a legközelebbi pont tényleg közel van!
    # ---------------------------------
    def slice_path_to_robot(self, path_msg: Path) -> Path:
        if not path_msg.poses:
            return path_msg

        frame = path_msg.header.frame_id
        rx, ry = self.get_robot_xy_in_frame(frame)
        if rx is None or ry is None:
            return path_msg

        best_i = 0
        best_d2 = float("inf")

        # Megkeressük a legközelebbi pose-t
        for i, ps in enumerate(path_msg.poses):
            dx = float(ps.pose.position.x) - rx
            dy = float(ps.pose.position.y) - ry
            d2 = dx * dx + dy * dy
            if d2 < best_d2:
                best_d2 = d2
                best_i = i

        best_d = math.sqrt(best_d2)

        # VÉDELEM: ha a legközelebbi pont túl messze van, nem vágunk!
        # Ez sokszor megszünteti a falnak-menést replannelésnél.
        if best_d > self.max_slice_dist:
            return path_msg

        # Ha túl kevés pont maradna, akkor inkább nem vágjuk
        if len(path_msg.poses) - best_i < 10:
            return path_msg

        out = Path()
        out.header = path_msg.header
        out.poses = path_msg.poses[best_i:]
        return out

    # ---------------------------------
    # Eldönti, hogy preempteljünk-e
    # ---------------------------------
    def should_preempt(self, new_path: Path) -> bool:
        now = self.now_sec()

        # túl hamar? ne cancel-eljünk folyamatosan
        if (now - self._last_goal_sent_time) < self.min_preempt_dt:
            return False

        # ha a replanned path eleje sokat változott -> igen
        if self.path_prefix_changed(new_path):
            return True

        # különben nem preemptelünk (egyszerűség)
        return False

    # ---------------------------------
    # FollowPath goal küldése
    # ---------------------------------
    def send_path_as_goal(self, path_msg: Path):
        self.get_logger().info(f"FollowPath goal küldése, poses={len(path_msg.poses)}")

        goal_msg = FollowPath.Goal()
        goal_msg.path = path_msg

        self._goal_active = True

        # preempt szűréshez elmentjük, mikor küldtük
        self._last_goal_sent_time = self.now_sec()

        # elmentjük a path elejét is (dinamikus változás figyeléshez)
        self._last_prefix_pts = self.path_prefix_xy(path_msg, self.compare_poses_n)

        future = self._client.send_goal_async(goal_msg)
        future.add_done_callback(self.goal_response_callback)

    # ---------------------------------
    # Path callback: ide érkezik minden új path
    # ---------------------------------
    def path_callback(self, msg: Path):
        if not msg.poses:
            self.get_logger().warn("Nincs Path message...")
            return

        # 1) Slice-oljuk (biztonságosan)
        msg2 = self.slice_path_to_robot(msg)

        # 2) Ha már a cél közelében vagyunk, ne küldjünk új goal-t
        if self.robot_close_to_path_goal(msg2):
            self.get_logger().info("Robot már cél közelében, új Path ignorálva.")
            return

        # 3) Ha túl rövid, nem küldjük
        if len(msg2.poses) < 10:
            self.get_logger().warn("Túl rövid path (slice után), nem küldöm FollowPath-nek.")
            return

        # 4) Ha fut goal: csak akkor preempteljünk, ha tényleg változott (és nem túl gyakran)
        if self._goal_active and self._goal_handle is not None:

            # ha cancel folyamatban van, nem csinálunk semmit
            if self._cancel_in_progress:
                return

            if not self.should_preempt(msg2):
                return

            # elmentjük az új path-ot, és cancel-eljük a régit
            self._pending_path = msg2
            self._cancel_in_progress = True
            self.get_logger().info("Új replanned Path jött - cancel régi FollowPath...")

            cancel_future = self._goal_handle.cancel_goal_async()
            cancel_future.add_done_callback(self.cancel_done_callback)
            return

        # 5) Ha nincs aktív goal, küldjük azonnal
        self.get_logger().info(
            f"Path megérkezett poses={len(msg.poses)}, küldés Nav2 felé (slice után {len(msg2.poses)})"
        )
        self.send_path_as_goal(msg2)

    # ---------------------------------
    # Cancel kész callback
    # ---------------------------------
    def cancel_done_callback(self, future):
        _ = future.result()

        self._cancel_in_progress = False
        self._goal_active = False
        self._goal_handle = None

        # Ha volt eltárolt új path, azt most elküldjük
        if self._pending_path is not None:
            p = self._pending_path
            self._pending_path = None
            self.get_logger().info("Cancel kész - küldöm az új replanned path-ot.")
            self.send_path_as_goal(p)

    # ---------------------------------
    # Goal response callback
    # ---------------------------------
    def goal_response_callback(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().warn("FollowPath cél elutasítva...")
            self._goal_active = False
            self._goal_handle = None
            return

        self._goal_handle = goal_handle
        self.get_logger().info("FollowPath cél elfogadva, várunk a resultra...")

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    # ---------------------------------
    # Result callback
    # ---------------------------------
    def result_callback(self, rf):
        _ = rf.result().result
        self.get_logger().info("FollowPath befejezte a működést.")

        self._goal_active = False
        self._goal_handle = None


def main(args=None):
    rclpy.init(args=args)
    node = Nav2PathClient()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()