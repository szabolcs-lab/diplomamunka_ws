import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Header
from nav_msgs.msg import MapMetaData
import os
import numpy as np
from ament_index_python.packages import get_package_share_directory

class MapPublication(Node):
    
    """
    A node egy OccupancyGridet fog publikálni a 'map' topicon, ami egy csv-ből betöltött rácstérkép
    """
    
    def __init__(self):
        super().__init__('map_publication')
        
        self.get_logger().info('Map publication node elindult....')
        
        # Paraméter beolvasás
        self.declare_parameter('map_file', '')
        map_file = self.get_parameter('map_file').get_parameter_value().string_value
        
        # CSV beolvasás int8 mátrixként     
        self.get_logger().info(f"Occupany grid betöltés a : {map_file} -ból")
        self.grid = np.loadtxt(map_file, delimiter=',').astype(np.int8)
       
        # Ezzel biztosítjuk, hogy minden node mindig megkapja a legfrissebb állapotot
        # Ha egy robot később indul, akkor azonnal megkapja a legfrissebb térképet
        qos = QoSProfile(depth=1) # Csak az utolsó üzenet marad a bufferben
        qos.reliability = ReliabilityPolicy.RELIABLE # Minden üzenet megérkezik a feliratkozóhoz
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL # Amikor egy új subscriber csatlakozik, megkapja a legutoljára publikált térképet
        
        # OccupancyGrid típusú üzenetet publikál a 'map' topicon
        self.map_publisher = self.create_publisher(OccupancyGrid, 'map', qos)
        
        # OccupancyGrid üzenet metaadatait állítjuk be
        self.map_msg = OccupancyGrid() # Létrehozunk egy OccupancyGrid objektumot, ami az üzenet lesz
        self.map_msg.header = Header() # Létrehozunk egy header-t
        self.map_msg.header.frame_id = 'map' # Frame_id az map lesz. Ez azt jelenti, hogy az egész térkép a map koordinátarendszerben lesz értelmezve
        
        # Ez fogja tartalmazni a térkép statikus leírását: méret, felbontás, eredet
        self.map_msg.info = MapMetaData()
        self.map_msg.info.height = self.grid.shape[0] # Hány sorból áll a rács (Y irány) 
        self.map_msg.info.width = self.grid.shape[1] # Hány oszlopból áll a rács (Y irány)
        self.map_msg.info.resolution = 0.1 # Egy cella 0.1 méter
        
        # Beállítjuk a térkép bal alsó sarkának világkoordinátáját; a gridet tükrözzük az OccuapancyGridre
        # teljes térkép fizikai szélessége /2 -> koordinátarendszer középpontját állítjuk a térkép közepére
        self.map_msg.info.origin.position.x = - self.map_msg.info.width * self.map_msg.info.resolution / 2.0
        # teljes térkép fizikai magassága /2 -> koordinátarendszer középpontját állítjuk a térkép közepére 
        self.map_msg.info.origin.position.y = - self.map_msg.info.height * self.map_msg.info.resolution / 2.0
        # A térkép a világ alapsíkjára kerül; térkép magasságát adjuk meg a világban 
        self.map_msg.info.origin.position.z = 0.0
        self.map_msg.info.origin.orientation.w = 1.0 # nincs elforgatás
        
        # Térkép celláit átalakítjuk OccupancyGrid formátummá, amit a ROS vár (0 - szabad, 1 - foglalt). Először 1D listává alakítjuk, majd szorozzuk 100-zal
        # Lapítás után a 0 mező a szabad lesz, 100 az akadály
        self.map_msg.data = (self.grid.flatten() * 100).tolist()
        
        # Másodpercenként, publikálja az OccupancyGridet amit a SLAM, a path planning, az RViz fel tud használni
        self.timer = self.create_timer(1.0, self.publish_map)
        
        self.get_logger().info('Map publication node inicializálva....')
        
    def publish_map(self):
        # időbélyeget állítjuk be a szinkronizáció miatt és ahhoz, hogy a RViz és Gazebo helyesen működjön
        #pl: több szenzor adatait kombináljuk, az időbélyeg alapján lehet összekapcsolni; Ha nincs időbélyeg, az üzenet rossz időben jelenhet meg
        self.map_msg.header.stamp = self.get_clock().now().to_msg()
        self.map_publisher.publish(self.map_msg)
        self.get_logger().info('A map üzenet publikálva...')
        
def main(args=None):
    rclpy.init(args=args)
    node = MapPublication()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()
