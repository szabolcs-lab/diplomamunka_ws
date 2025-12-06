import os
import math
import random
import subprocess # külső parancs futtatásához kell

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from ament_index_python.packages import get_package_share_directory


class DynamicObstacleSpawner(Node):
    '''
        A node vár egy random ideig, közben kap egy útvonalat (planned_path_dilated), majd a kapott út felénél kiválaszt egy pontot,
        ott az útra ráhelyez egy akadályt Gazebo-ban és publikálja a /dynamic_obstacle topicra, 
        hogy más node-ok is tudjanak róla.
    '''
    def __init__(self):
        super().__init__('dynamic_obstacle_spawner')

        # paraméterek beolvasása
        self.min_delay = self.declare_parameter('min_delay', 17.0).value
        self.max_delay = self.declare_parameter('max_delay', 24.0).value
        self.world_name = self.declare_parameter('world_name', 'custom_world').value
        self.obstacle_name = self.declare_parameter('obstacle_name', 'dynamic_obstacle').value
        self.obstacle_z = self.declare_parameter('obstacle_z', 0.0).value

        # Modell elérési út (simulation_resources_pkg/worlds/dynamic_box.sdf)
        pkg_share = get_package_share_directory('simulation_resources_pkg')
        self.model_path = os.path.join(pkg_share, 'worlds', 'dynamic_box.sdf')
        
        qos_obstacle = QoSProfile(depth=10) # Buffer méret 10
        qos_obstacle.reliability = ReliabilityPolicy.RELIABLE # ezzel garantáljuk, hogy egy üzenet sem fog elveszni
        qos_obstacle.durability = DurabilityPolicy.TRANSIENT_LOCAL # amikor egy node később csatlakozik megkapja az utlsó üzenetet

        # Random késleltetés min_delay = 17 és max_ delay = 24 között
        self.delay = random.uniform(self.min_delay, self.max_delay)

        # Állapot
        self.current_path = None # még nincs beolvasott útvonal
        self.spawned = False # még nincs spawnolva akadály
        self.start_time = self.get_clock().now() # indulási időt elmentjük

        # Feliratkozás a planned_path_dilated útvonalra
        self.create_subscription(Path, 'planned_path_dilated', self.path_callback, 10)
        
        # itt publikáljuk, majd az akadály pozícióját
        self.obstacle_pub = self.create_publisher(PoseStamped, 'dynamic_obstacle', qos_obstacle)

        # időzítő, fél másodpercenként meghívódik a timer_callback
        self.create_timer(0.5, self.timer_callback)

    # Callbacks
    def path_callback(self, msg: Path):
        # ha van útvonal a feliratokzott topicon, akkor elmentjük
        if not msg.poses:
            return
        self.current_path = msg

    def timer_callback(self):
        # ha már van akadály amit betettünk, már nem tesszük be újra
        if self.spawned:
            return

        # idő ellenőrzés, ssak akkor engedi tovább a kódot, ha letelt a random késleltetés
        elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        if elapsed < self.delay:
            return
        # megnézzük, hogy van-e útvonal, ha nincs akkor nem csinálunk semmit
        if self.current_path is None:
            return

        # kiválasztjuk az útvonal közepét a függvény meghívásával
        pose = self.middle_point_on_the_path(self.current_path)
        
        # ellnőrizzük, hogy talalátunk-e ilyen pontot
        if pose is None:
            self.get_logger().error('Nincs használható pont a path közepén...')
            return

        # ha van ilyen pont, akkor elmentjük a koordinátáit
        x = pose.pose.position.x
        y = pose.pose.position.y
        
        # átadjuk az x és y pontokat a spawn_obstacle függvénynek, ami, majd spawnolj az akadályt
        self.spawn_obstacle(x, y, self.obstacle_z)
        self.publish_dynamic_obstacle(x, y, self.obstacle_z)
        
        self.spawned = True

    # A függvény a path teljes hosszát kiszámolja, majd a kb. a felénél lévő pozíciót visszaadja
    def middle_point_on_the_path(self, path_msg: Path):
        # a path üzenetben kapot pontokat kivesszük egy változóba
        poses = path_msg.poses
        
        # ha ez a hossz kisebb, mint 2, akkor nem térünk vissza semmivel
        if len(poses) < 2:
            return None

        # kiszámoljuk a tlejes út hosszát, úgy, hogy visszamegyünk a pontokon és az egyes pontok közötti távolságot szummázzuk
        total_length = 0.0
        last = poses[0].pose.position
        
        for i in range(1, len(poses)):
            p = poses[i].pose.position
            
            # egymást követő pontok közötti távolság számítása
            direction_x = p.x - last.x
            direction_y = p.y - last.y
            total_length += math.hypot(direction_x, direction_y)
            last = p

        # itt ellenőrízzóük, hogy az út hossza nem 'nulla', ha kvázi nulla, akkor az utolsó pontot adjuk vissza
        if total_length <= 1e-6:
            return poses[-1]

        # felezzük az út hosszát
        half_length = total_length / 2.0

        # a ciklus addig megy, amíg az előzőleg szummázott távolságnál el nem éri a felét
        sum_distance = 0.0
        last = poses[0].pose.position
        
        for i in range(1, len(poses)):
            p = poses[i].pose.position
            
            # egymást követő pontok közötti távolság számítása
            direction_x = p.x - last.x
            direction_y = p.y - last.y     
            segment_length = math.hypot(direction_x, direction_y)
            
            sum_distance += segment_length
            
            if sum_distance >= half_length:
                return poses[i]
            last = p

        return poses[-1]

    # ezzel függvénnyel végezzük le spawn-olást
    def spawn_obstacle(self, x: float, y: float, z: float):
        # ellenőrizzük, hogy a model az adott helyen van-e
        if not os.path.exists(self.model_path):
            self.get_logger().error(f'Az sdf fájl ami az akadály modeljét tartalmazza nem található: {self.model_path}')
            return

        # itt állítjuk össze a Gazebo ros_gz_sim objektum-spawn parancsát
        # ugyanazt teesszük, mint cmd-ben : 
        # ros2 run ros_gz_sim create -world 'self.world_name' -file 'sself.model_path' -name 'self.obstacle_name' -x str(x) -y str(y) -z str(z)
        cmd = [
            'ros2', 'run', 'ros_gz_sim', 'create',
            '-world', self.world_name,
            '-file', self.model_path,
            '-name', self.obstacle_name,
            '-x', str(x),
            '-y', str(y),
            '-z', str(z),
        ]

        self.get_logger().info('Akadály spawn-olása...')
        
        # összefűzzük a cmd lista elemeit 
        # # ros2 run ros_gz_sim create -world 'self.world_name' -file 'sself.model_path' -name 'self.obstacle_name' -x str(x) -y str(y) -z str(z)
        self.get_logger().info(' '.join(cmd))

        # itt futtatjuk a ROS2 spawn parancsot úgy, mintha a terminálba írtuk volna be, a subprocess.run arra való, hogy elindítsunk külső parancsokat
        result = subprocess.run(cmd, capture_output=True, text=True)
        if result.returncode == 0:
            self.get_logger().info('Az akadály spawn-olása suikerült...')
        else:
            self.get_logger().error(f'Spawn-olás nem sikerült...')
    
    
    # létrehozunk egy PoseStamped üzenetet, amit publikálunk, majd a dynamic_obstacle topicra       
    def publish_dynamic_obstacle(self, x: float, y: float, z: float):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'   # a path is map frame-ben van

        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = z

        msg.pose.orientation.w = 1.0

        self.obstacle_pub.publish(msg)
        self.get_logger().info(f'Az akadály publikálva a /dynamic_obstacle topcin, a következő pontokra (x={x:.2f}, y={y:.2f})')


def main(args=None):
    rclpy.init(args=args)
    node = DynamicObstacleSpawner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
