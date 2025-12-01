import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Header
from nav_msgs.msg import MapMetaData
import numpy as np

class MapPublication(Node):
    
    """
    A node egy OccupancyGridet fog publikálni a 'map' topicon, amit egy csv-ből olvas be
    """
    
    def __init__(self):
        super().__init__('map_publication')
        
        self.get_logger().info('Map publication node indul....')
        
        # Beolvassuk a paramétert, amit launch fájlban teszünk össze, az indítás során megadott paraméter és a PathJoinSubstitution összefüzésével
        self.declare_parameter('map_file', '')
        map_file = self.get_parameter('map_file').get_parameter_value().string_value
        
        # map_file-ban lévő elérési út mutatja meg, hogy hol a csv. Ezt a csv-t beolvassuk egy numpy tömbként és letároljuk 
        self.grid = np.loadtxt(map_file, delimiter=',').astype(np.int8)
       
        # A qos beállításával oldjuk meg, hogy minden node ami használja, majd a map topicot, az mindig megkapja a legfrissebb állapotot
        qos = QoSProfile(depth=1) # Ezzel biztosítjuk, hogy a bufferben mindig a legutolsó publikált üzenet legyen
        qos.reliability = ReliabilityPolicy.RELIABLE # Ezzel garantáljuk, hogy mindenki aki feliratkozik erre a topicra az minden üzenetet megkapjon
        # Amikor egy új node iratkozik fel a topicra, akkor ez a sor biztosítja, hogy megkapja a legutoljára publikált térképet
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL 
        
        # A 'map' topicon publikálunk egy OccupancyGrid típusú üzenetet
        self.map_publisher = self.create_publisher(OccupancyGrid, 'map', qos)
        
        # Létrehozzuk az OccupancyGrid típusú üzenetetv és beállítjuk az értékeit
        self.map_msg = OccupancyGrid() # Létrehozunk egy OccupancyGrid objektumot
        self.map_msg.header = Header() # Létrehozunk egy header-t
        self.map_msg.header.frame_id = 'map' # Frame_id az map lesz. Itt mondjuk meg, hogy a térkép a map nevű koordinátarendszerben lesz
        
        # Ez fogja tartalmazni a térkép adatait: méret, felbontás, forgatás, térkép elhelyzekedése a világ síkjában
        self.map_msg.info = MapMetaData()
        self.map_msg.info.height = self.grid.shape[0] # Beállítjuk, hogy mennyi sorból fog állni a rács, az y-t adjuk meg 
        self.map_msg.info.width = self.grid.shape[1] # Beállítjuk, hogy mennyi oszlopból fog állni a rács, az x-t adjuk meg
        self.map_msg.info.resolution = 0.1 # Beállítjuk egy cella méretét méterben
        
        # Beállítjuk a térkép bal alsó sarkának világkoordinátáját, a gridet tükrözzük
        # A térkép teljes fizikai szélességét  osztjuk 2-vel, mert a térkép közepét a koordinátarendszer origójához igazítjuk.
        self.map_msg.info.origin.position.x = - self.map_msg.info.width * self.map_msg.info.resolution / 2.0
        
        # A térkép teljes fizikai magasságát osztjuk 2-vel, mert a térkép közepét a koordinátarendszer origójához igazítjuk.
        self.map_msg.info.origin.position.y = - self.map_msg.info.height * self.map_msg.info.resolution / 2.0
        
        # A térképet a világ függőlegsen helyezzük el vagyis azt mondjuk meg, hogy a térkép a világ síkjához képest hol helyzkedjen el
        self.map_msg.info.origin.position.z = 0.0
        self.map_msg.info.origin.orientation.w = 1.0 # Nem forgatjuk el a térképet
        
        # Térkép celláit átalakítjuk OccupancyGrid formátumuvá, ahol a 0-ás a szabad, 1-es a foglalt mező lesz. Aztán listává alakítjuk, majd szorozzuk 100-zal
        # A szorzást követően a 0-ás mező továbbra is szabad lesz, 100-as mező pedig az akadályt fogja jelölni
        self.map_msg.data = (self.grid.flatten() * 100).tolist()
        
        # Másodpercenként publikáljuk az OccupancyGridet azért, hogy, mind a path_planner és mind az RViz tudja használni
        self.timer = self.create_timer(1.0, self.publish_map)
        
        self.get_logger().info('Map publication node inicializálva....')
        
    def publish_map(self):
        # Beállítjuk az időbélyeget a szinkronizáció miatt, hogy az Rviz és a Gazebo jól működjönk
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
