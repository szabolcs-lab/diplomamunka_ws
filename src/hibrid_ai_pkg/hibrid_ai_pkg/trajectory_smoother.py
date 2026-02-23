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

        self.path_subscriber = self.create_subscription(Path, "/planned_path_dilated", self.path_callback, qos_path)

        # params-ra elég sima QoS, mert a trainer amúgy is folyamatosan küldi
        self.params_subscriber = self.create_subscription(Float32MultiArray, "/smoother_params", self.params_callback, 10)

        self.path_publisher = self.create_publisher(Path, "/planned_path_smoother", qos_path)

        self.get_logger().info(f"TrajectorySmoother elindult: /planned_path_dilated - /planned_path_smoother")

    def params_callback(self, msg: Float32MultiArray):
        # várjuk: [offset, smooth]
        if len(msg.data) < 2:
            return

        # clamp (biztonság)
        offset_m = float(msg.data[0])
        smoothimg_strength = float(msg.data[1])

        if offset_m > 0.2:
            offset_m = 0.2
        if offset_m < -0.2:
            offset_m = -0.2

        if smoothimg_strength < 0.0:
            smoothimg_strength = 0.0
        if smoothimg_strength > 1.0:
            smoothimg_strength = 1.0

        self.offset_m = offset_m
        self.smooth_strength = smoothimg_strength

        # self.get_logger().info(f"params: offset={self.offset_m:.3f} smooth={self.smooth_strength:.2f}")

    def path_callback(self, msg: Path):
        if len(msg.poses) < 3:
            self.path_publisher.publish(msg)
            return

        points_xy = [(pose.pose.position.x, pose.pose.position.y) for pose in msg.poses]

        # offset ráhúzása
        if abs(self.offset_m) > 1e-6:
            points_xy = self.apply_offset(points_xy, self.offset_m)

        # simítás (0/1/2 iter)
        smoothing_iter  = 0
        if self.smooth_strength < 0.33:
            smoothing_iter = 0
        elif self.smooth_strength < 0.66:
            smoothing_iter = 1
        else:
            smoothing_iter = 2

        if smoothing_iter > 0:
            points_xy = self.chaikin_smooth(points_xy, smoothing_iter)

        # vissza Path üzenetbe
        smoothed_path_msg  = Path()
        smoothed_path_msg .header = msg.header

        for x, y in points_xy:
            pose_stamped = PoseStamped()
            pose_stamped.header = smoothed_path_msg .header
            pose_stamped.pose.position.x = float(x)
            pose_stamped.pose.position.y = float(y)
            pose_stamped.pose.position.z = 0.0
            pose_stamped.pose.orientation.w = 1.0
            smoothed_path_msg .poses.append(pose_stamped)
         
        #use_sim_time miatt tettem be!    
        now = self.get_clock().now().to_msg()
        smoothed_path_msg .header.stamp = now
        for pose in smoothed_path_msg.poses:
            pose.header.stamp = now

        self.path_publisher.publish(smoothed_path_msg )

    def apply_offset(self, points_xy, offset_m: float):
        """
        - minden ponthoz becsüljük a tangens irányt (előző-következő)
        - ebből normált számolunk (balra)
        - pontot eltoljuk normál irányba
        """
        number_points = len(points_xy)
        out_ofset_points = []

        for i in range(number_points):
            if i == 0:
                x0, y0 = points_xy[i]
                x1, y1 = points_xy[i + 1]
            elif i == number_points - 1:
                x0, y0 = points_xy[i - 1]
                x1, y1 = points_xy[i]
            else:
                x0, y0 = points_xy[i - 1]
                x1, y1 = points_xy[i + 1]
                
            tangens_x = x1 - x0
            tangens_y = y1 - y0

            tangens_len = math.hypot(tangens_x, tangens_y)
            if tangens_len < 1e-6:
                out_ofset_points.append(points_xy[i])
                continue

            tangens_x /= tangens_len
            tangens_y /= tangens_len

            # bal oldali normál
            normal_vectx = -tangens_y
            normal_vecty = tangens_x

            currentx, currenty = points_xy[i]
            out_ofset_points.append((currentx + offset_m * normal_vectx, currenty + offset_m * normal_vecty))

        return out_ofset_points
        
    def chaikin_smooth(self, eredeti_pontok, hany_szor_simitunk: int):
        
        """
        Chaikin-simítás:
            minden szakaszt két pontra bont:
                Q = 0.75*P0 + 0.25*P1
                R = 0.25*P0 + 0.75*P1
            ettől lekerekedik az útvonal
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
