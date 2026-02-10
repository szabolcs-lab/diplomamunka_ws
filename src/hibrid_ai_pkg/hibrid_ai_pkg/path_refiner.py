import math
import rclpy
from rclpy.node import Node

from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32MultiArray


class PathRefiner(Node):
    """
    Egyszerű útvonal finomító (BSc-barát).

    Input:
      - /planned_path_dilated  (nav_msgs/Path)  -> D* Lite útvonal
      - /refiner_params        (Float32MultiArray) -> [offset_m, smooth_strength]

    Output:
      - /planned_path_refined  (nav_msgs/Path)

    Mit csinál:
      1) offset: az útvonalra "oldalra tolás" (bal/jobb), méterben
      2) smooth: simítás erősség (0..1), ebből 0/1/2 iteráció lesz

    Megjegyzés:
      - Itt nem számolunk orientációt, csak pontokat publikálunk.
      - A Nav2 controller ezt így is tudja követni.
    """

    def __init__(self):
        super().__init__("path_refiner")

        # topicok (paramként, hogy könnyű legyen launchból állítani)
        self.declare_parameter("path_in", "/planned_path_dilated")
        self.declare_parameter("path_out", "/planned_path_refined")
        self.declare_parameter("params_topic", "/refiner_params")

        self.path_in = str(self.get_parameter("path_in").value)
        self.path_out = str(self.get_parameter("path_out").value)
        self.params_topic = str(self.get_parameter("params_topic").value)

        # aktuális paraméterek
        self.offset_m = 0.0
        self.smooth_strength = 0.0

        # QoS a path-ra: transient_local, hogy későn induló node is kapjon utolsót
        qos_path = QoSProfile(depth=1)
        qos_path.reliability = ReliabilityPolicy.RELIABLE
        qos_path.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self.sub_path = self.create_subscription(Path, self.path_in, self.cb_path, qos_path)

        # params-ra elég sima QoS, mert a trainer amúgy is folyamatosan küldi
        self.sub_params = self.create_subscription(Float32MultiArray, self.params_topic, self.cb_params, 10)

        self.pub_path = self.create_publisher(Path, self.path_out, qos_path)

        self.get_logger().info(f"PathRefiner indul: {self.path_in} -> {self.path_out}")
        self.get_logger().info(f"Param topic: {self.params_topic}  (data=[offset, smooth])")

    def cb_params(self, msg: Float32MultiArray):
        # várjuk: [offset, smooth]
        if len(msg.data) < 2:
            return

        # clamp (biztonság)
        off = float(msg.data[0])
        sm = float(msg.data[1])

        if off > 0.2:
            off = 0.2
        if off < -0.2:
            off = -0.2

        if sm < 0.0:
            sm = 0.0
        if sm > 1.0:
            sm = 1.0

        self.offset_m = off
        self.smooth_strength = sm

        # debug: ha akarod, hagyd bent
        # self.get_logger().info(f"params: offset={self.offset_m:.3f} smooth={self.smooth_strength:.2f}")

    def cb_path(self, msg: Path):
        if len(msg.poses) < 3:
            self.pub_path.publish(msg)
            return

        pts = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]

        # 1) offset ráhúzása (szakaszonként normál)
        if abs(self.offset_m) > 1e-6:
            pts = self.apply_offset(pts, self.offset_m)

        # 2) simítás (0/1/2 iter)
        iters = 0
        if self.smooth_strength < 0.33:
            iters = 0
        elif self.smooth_strength < 0.66:
            iters = 1
        else:
            iters = 2

        if iters > 0:
            pts = self.chaikin_smooth(pts, iters)

        # 3) vissza Path üzenetbe
        out = Path()
        out.header = msg.header

        for x, y in pts:
            ps = PoseStamped()
            ps.header = out.header
            ps.pose.position.x = float(x)
            ps.pose.position.y = float(y)
            ps.pose.position.z = 0.0
            ps.pose.orientation.w = 1.0
            out.poses.append(ps)

        self.pub_path.publish(out)

    def apply_offset(self, pts, offset_m: float):
        """
        Egyszerű ötlet:
        - minden ponthoz becsüljük a tangens irányt (előző->következő)
        - ebből normált számolunk (balra)
        - pontot eltoljuk normál irányba
        """
        n = len(pts)
        out = []

        for i in range(n):
            if i == 0:
                x0, y0 = pts[i]
                x1, y1 = pts[i + 1]
                tx = x1 - x0
                ty = y1 - y0
            elif i == n - 1:
                x0, y0 = pts[i - 1]
                x1, y1 = pts[i]
                tx = x1 - x0
                ty = y1 - y0
            else:
                x0, y0 = pts[i - 1]
                x1, y1 = pts[i + 1]
                tx = x1 - x0
                ty = y1 - y0

            tlen = math.hypot(tx, ty)
            if tlen < 1e-6:
                out.append(pts[i])
                continue

            tx /= tlen
            ty /= tlen

            # bal oldali normál
            nx = -ty
            ny = tx

            x, y = pts[i]
            out.append((x + offset_m * nx, y + offset_m * ny))

        return out

    def chaikin_smooth(self, pts, iters: int):
        """
        Chaikin-simítás:
        - minden szakaszt két pontra bont:
          Q = 0.75*P0 + 0.25*P1
          R = 0.25*P0 + 0.75*P1
        - ettől "lekerekedik" az útvonal
        """
        out = pts
        for _ in range(iters):
            if len(out) < 3:
                return out

            new_pts = [out[0]]
            for i in range(len(out) - 1):
                p0 = out[i]
                p1 = out[i + 1]

                qx = 0.75 * p0[0] + 0.25 * p1[0]
                qy = 0.75 * p0[1] + 0.25 * p1[1]

                rx = 0.25 * p0[0] + 0.75 * p1[0]
                ry = 0.25 * p0[1] + 0.75 * p1[1]

                new_pts.append((qx, qy))
                new_pts.append((rx, ry))

            new_pts.append(out[-1])
            out = new_pts

        return out


def main(args=None):
    rclpy.init(args=args)
    node = PathRefiner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
