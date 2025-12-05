import numpy as np
from math import sqrt, atan2, cos, sin


class Node:
    def __init__(self, y, x):
        self.y = y  # sorindex (függőleges)
        self.x = x  # oszlopindex (vízszintes)
        self.parent = None
        self.cost = 0.0

# euklidészi távolság két pont között
def node_distance(n1, n2):
    return sqrt((n1.x - n2.x)**2 + (n1.y - n2.y)**2)


class RRTStar:
    def __init__(self, grid, start, goal, max_iter=2000, step_len=3, goal_sample_rate=0.05, rewire_radius=6):
        self.grid = grid
        self.height, self.width = grid.shape
        self.start = Node(start[0], start[1])  
        self.goal = Node(goal[0], goal[1]) 
        # ennyi iterációból próbáljuk elérni a célt, kvázi ez egy biztonsági határ, hogy az algoritmus ne fusson sokáig, a nagysággal egyenes arányosságban nő a pontosság     
        self.max_iter = max_iter 
        self.step_len = step_len # ekkora ugásokkal növekszik a fa, ha ez kisebb annál finomabb lesz a lépés
        self.goal_sample_rate = goal_sample_rate # x% eséllyel a célpontot választja mintának, gyorsítás a cél felé
        self.rewire_radius = rewire_radius # ha van új csúcs, akkor az adott sugáron belül próbáljuk átkötni a szomszédos csomópontokat a jobb útvonal miatt.
        self.tree = [] # itt fog táorlódni az épülő fa
        self.processed_nodes = 0 #majd a metrikáknál használjuk fel, számlalóként használjuk, hogy mennyi csomópontot dolgozott fel az algoritmus

    # fő RRT* algoritmus
    def plan(self):
        # fába betesszük az első elemet, amit a start node
        self.tree = [self.start]

        # elindul a ciklus max_iter = 2000 -ig, ennyi ideje van úgy fát építeni, hogy elérje a célt
        for i in range(self.max_iter):
            rnd = self.sample() # véletlen mintavétel a térből ez segít, hogy a fa nőljön és elérje a célt
            nearest_node = self.get_nearest(rnd) # megkeressük, hogy melyik csomópont van a legközelebb az előbb vett véletlen ponthoz.
            new_node = self.steer(nearest_node, rnd) # majd ebből a legközelebbi pontból egy step = 3 lépést teszütnk felé

            # ha akdály az az él, akkor ugrunk egy iterációt és ezt a lépést kihagyjuk
            if not self.is_free(new_node, nearest_node):
                continue

            # az új pontnak beállítjuk a szülőjét a legközelebbi pontra
            new_node.parent = nearest_node
            # kiszámoljuk az új pontnaj a hosszát, költségét a starttól
            new_node.cost = nearest_node.cost + node_distance(nearest_node, new_node)

            # olyam pontokat keresünk, amelyik rewire_radius sugarú körön belül van
            near_nodes = self.get_near_nodes(new_node)
            # a közelben lévő csomópontok közül megnézzük azt, hogy kihez lenne a legolcsóbb csatlakozni
            new_node = self.choose_parent(new_node, near_nodes)
            # az új node-ot felvesszük a fába
            self.tree.append(new_node)
            # Ha a csomópontok közül bármelyik rövidebb úton elérhető az új csomóponton keresztül, akkor változatjuk a szülőt, költséget és az útvonalat
            self.rewire(new_node, near_nodes)
            
            self.processed_nodes += 1 # csomópont feldolgozáshoz számlaló , ez a metrikához kell

            # elkészítjük az útvonalat, ha az új pont közel van a célhoz
            if node_distance(new_node, self.goal) < self.step_len:
                return self.extract_path(new_node)

        return None

    # Véletlen mintavétel (y,x) koordinátákban
    def sample(self):
        # 5% eséllyel nem a véletlen pontot, hanem a célpontot adja vissza
        if np.random.rand() < self.goal_sample_rate:
            return self.goal
        
        # véletlen koordináta választás (y,x) (sor, oszlop) (height, width)
        height_random = np.random.uniform(0, self.height - 1)
        width_random = np.random.uniform(0, self.width - 1)
        
        return Node(height_random, width_random)

    # itt nézzük meg, hogy a véletlen ponthoz, a fa meelyik csomópontja van a legközelebb
    def get_nearest(self, rnd):
        return min(self.tree, key=lambda n: node_distance(n, rnd))

    def steer(self, from_node, to_node):
        d = node_distance(from_node, to_node)
        
        if d < 1e-6: # 1e-6 = 0.000001
            return from_node
        
        # meghatározza a pontos irányszöget from_node-tól a to_node-ig, egy irányt ad
        theta = atan2(to_node.y - from_node.y, to_node.x - from_node.x)
        # vagy a teljes távolságot vagy a step_len-t lépjük, kettő közül a minimumot
        dist = min(self.step_len, d)
        # elmozdulás a síkon, koordinátarendszer miatt, ha y irányba mozdulunk - sin és, ha x irányba mozudlunk - cos
        new_y = from_node.y + dist * sin(theta)
        new_x = from_node.x + dist * cos(theta)
        
        return Node(new_y, new_x)

    # Ellenőrzzük, hogy a node és a szülője közötti él szabad-e
    def is_free(self, node, from_node=None):
        # a lebegópontos koordinátákat egésszé alakítja
        grid_x = int(round(node.x))
        grid_y = int(round(node.y))
        
        # megnézzük, hogy a griden belül vagyunk-e
        if grid_x < 0 or grid_y < 0 or grid_x >= self.width or grid_y >= self.height:
            return False
        
        # esteg akadály-e
        if self.grid[grid_y, grid_x] == 1:
            return False

        # majd a starthoz vagy a célhoz kell, itt csak pontot ellenőrizzük
        if from_node is None:
            return True

        # egy egyenes mentén kiszámoljuk azt az utat ami a kiindulási és a célpont között van, ezt egyenletesen osztjuk fel és visszaadjuk az összes köztes pontot
        steps = int(max(abs(node.x - from_node.x), abs(node.y - from_node.y)))
        for i in range(steps + 1):
            t = i / max(1, steps)
            x = int(round(from_node.x + t * (node.x - from_node.x)))
            y = int(round(from_node.y + t * (node.y - from_node.y)))
            
            # megnézzük, hogy griden belül vagyunk-e
            if x < 0 or y < 0 or x >= self.width or y >= self.height:
                return False
            
            # megnézzük, hogy nem-e akadály
            if self.grid[y, x] == 1:
                return False
            
        return True

    # kiszedjük azokat a csomópontokat a fából, amelyek a rewire_radius távolságon belül vannak az új csomóponthoz képest
    def get_near_nodes(self, new_node):
        return [n for n in self.tree if node_distance(n, new_node) <= self.rewire_radius]

    # az új csomópontnak a közeli pontokból kiválasztjuk azt a legjobb szülőt, ahol starttól kezdve az összköltség minimális
    def choose_parent(self, new_node, near_nodes):
        # ha nincsenek közeli pontok, akkor az új node szülője a legközelebbi node marad
        if not near_nodes:
            return new_node

        # beállítjuk a min költségnek és a legjobb szülőnek az új node ejelnlegi költségét és szülőjét
        min_cost = new_node.cost
        best_parent = new_node.parent

        # végigmegxünk a node-okon egyesével és kiszámoljuk a költségüket az új node-hoz képest
        for near in near_nodes:
            cost = near.cost + node_distance(near, new_node)
            
            # ha ez a számított költség kisebb, mint a beállított min és az él nem ütközik akadályba, akkor beállítjuk az új értékeket a min-be és a best_parentbe
            if cost < min_cost and self.is_free(new_node, near):
                min_cost = cost
                best_parent = near

        # beállítjuk az új optomális szülőt és költéget
        new_node.parent = best_parent
        new_node.cost = min_cost
        
        return new_node

    # átdrótozás, beállítjuk a közeli csomópontok szülőjét az új csomópontra, ha az jobb költésget ad
    def rewire(self, new_node, near_nodes):
        for near in near_nodes:
            cost_through_new = new_node.cost + node_distance(new_node, near)
            if cost_through_new < near.cost and self.is_free(new_node, near):
                near.parent = new_node
                near.cost = cost_through_new

    # útvonal visszafejtése a céltól a startig
    def extract_path(self, node):
        path = []
        while node is not None:
            path.append((node.y, node.x)) 
            node = node.parent
        path.reverse()
        return path
