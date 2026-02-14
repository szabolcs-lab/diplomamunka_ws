import math
import rclpy
from rclpy.node import Node

from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32MultiArray


class TrajectorySmoother(Node):
    def __init__(self):
        super().__init__("trajectory_smoother")

        # topicok (paramként, hogy könnyű legyen launchból állítani)
        #self.declare_parameter("path_in", "/planned_path_dilated")
        #self.declare_parameter("path_out", "/planned_path_refined")
        #self.declare_parameter("params_topic", "/refiner_params")

        #self.path_in = str(self.get_parameter("path_in").value)
        #self.path_out = str(self.get_parameter("path_out").value)
        #self.params_topic = str(self.get_parameter("params_topic").value)

        # aktuális paraméterek
        self.offset_m = 0.0
        self.smooth_strength = 0.0

        # QoS a path-ra: transient_local, hogy későn induló node is kapjon utolsót
        qos_path = QoSProfile(depth=1)
        qos_path.reliability = ReliabilityPolicy.RELIABLE
        qos_path.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self.sub_path = self.create_subscription(Path, "/planned_path_dilated", self.cb_path, qos_path)

        # params-ra elég sima QoS, mert a trainer amúgy is folyamatosan küldi
        self.sub_params = self.create_subscription(Float32MultiArray, "/smoother_params", self.cb_params, 10)

        self.pub_path = self.create_publisher(Path, "/planned_path_smoother", qos_path)

        self.get_logger().info(f"PathRefiner indul: /planned_path_dilated -> /planned_path_smoother")

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

        # self.get_logger().info(f"params: offset={self.offset_m:.3f} smooth={self.smooth_strength:.2f}")

    def cb_path(self, msg: Path):
        if len(msg.poses) < 3:
            self.pub_path.publish(msg)
            return

        pts = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]

        # 1) offset ráhúzása
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
        - minden ponthoz becsüljük a tangens irányt (előző-következő)
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
        
    def chaikin_smooth(self, eredeti_pontok, hany_szor_simitunk: int):
        
        """
        Chaikin-simítás:
            - minden szakaszt két pontra bont:
                Q = 0.75*P0 + 0.25*P1
                R = 0.25*P0 + 0.75*P1
            - ettől lekerekedik az útvonal
        """
        
        # Másolatot készítek
        uj_pontok = eredeti_pontok.copy()
        
        #Ennyiszer ismételem
        for simitas_kor in range(hany_szor_simitunk):
            
            #Ha túl kevés pont, akkor stop
            if len(uj_pontok) < 3:
                return uj_pontok
            
            #Teljesen új pontlista kezdése
            ujabb_pontok = []
            
            # 1. pont mindig marad (start)
            ujabb_pontok.append(uj_pontok[0])
            
            # Minden egyenes szakaszra lesz két új pontt
            for i in range(len(uj_pontok) - 1):
                # Két végpont
                kezdo_x, kezdo_y = uj_pontok[i]     # P0 = kezdőőpont
                vege_x, vege_y = uj_pontok[i + 1]   # P1 = végpont
                
                # új pont Q (25% helyen)
                elso_uj_x = 0.75 * kezdo_x + 0.25 * vege_x
                elso_uj_y = 0.75 * kezdo_y + 0.25 * vege_y
                
                #új pont R (75% helyen)  
                masodik_uj_x = 0.25 * kezdo_x + 0.75 * vege_x
                masodik_uj_y = 0.25 * kezdo_y + 0.75 * vege_y
                
                # Hozzáadom a listához
                ujabb_pontok.append((elso_uj_x, elso_uj_y))
                ujabb_pontok.append((masodik_uj_x, masodik_uj_y))
            
            # Utolsó pont mindig marad (cél)
            ujabb_pontok.append(uj_pontok[-1])
            
            # Következő körnek ez lesz a kiindulá
            uj_pontok = ujabb_pontok
        
        return uj_pontok
    
        """ TESZTHEZ EGY MÁSIK VARIÁCIÓ!!!!!!!!!!!!!!!!!!!!!!!
        def laplace_smooth(self, eredeti_pontok, hany_szor_simitunk: int, mennyire_erosen: float = 0.5):      
                
            A robot szögletes, rángatózó útvonalát szépen lekerekíti,de a KIINDULÁSI és CÉLPONT nem mozdul el!
            
            Minden pontot megnéz: "Mit javasolnak a szomszédok?"
            Ha nagyon eltér, kicsit közelebb húzza hozzájuk.
            
            #new_x = x_curr + alpha * (avg_x - x_curr)  ===  p[i] + alfa * ( (p[i-1]+p[i+1])/2 - p[i] )
            
            # Másolat készítek azért, hogy ne buheráljam szét az eredetit
            uj_pontok = eredeti_pontok.copy()
            
            # Ennyiszer ismételem meg a simítást
            for simitas_kor in range(hany_szor_simitunk):
                
                # Ha túl kevés pont, nem csinálok semmit
                if len(uj_pontok) < 3:
                    return uj_pontok
                
                # új pont listát készítek
                ujabb_pontok = []
                
                # 1. pont MINDIG marad a helyén (start)
                ujabb_pontok.append(uj_pontok[0])
                
                # Középső pontok simítása
                for i in range(1, len(uj_pontok) - 1):
                    # Szomszédok koordinátái
                    bal_x, bal_y = uj_pontok[i - 1]      # Bal szomszéd
                    most_x, most_y = uj_pontok[i]        # Jelenlegi pont
                    jobb_x, jobb_y = uj_pontok[i + 1]    # Jobb szomszéd
                    
                    # Szomszédok átlaga
                    atlag_x = 0.5 * (bal_x + jobb_x)
                    atlag_y = 0.5 * (bal_y + jobb_y)
                    
                    # új pozi kicsit közelebb a szomszédok tanácsához
                    uj_x = most_x + mennyire_erosen * (atlag_x - most_x)
                    uj_y = most_y + mennyire_erosen * (atlag_y - most_y)
                    
                    ujabb_pontok.append((uj_x, uj_y))
                
                # Utolsó pont mindig marad (cél)
                ujabb_pontok.append(uj_pontok[-1])
                
                # Ez lesz a következő kör kiindulása
                uj_pontok = ujabb_pontok
                
            return uj_pontok
        """


def main(args=None):
    rclpy.init(args=args)
    node = TrajectorySmoother()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
