import math
from collections import defaultdict
import heapq

class DStarLite():
    '''
        A D* Lite algoritmus egy dinamikus útvonaltervező algoritmus, ami megtalálja a legrövidebb útvonalat és képes újratervezni
    '''
    def __init__(self, grid, start, goal):
        
        self.grid = grid    
        self.start = start
        self.start_last = self.start
        self.goal = goal
       
        # minden csomóponthoz beállítjuk a végtelenet
        self.g = defaultdict(lambda: math.inf)
        self.rhs = defaultdict(lambda: math.inf)
        
        self.g[self.goal] = math.inf
        self.rhs[self.goal] = 0.0
        
        self.k_m = 0.0
        
        self.U = []
        self.open_dict = {}
        self.insert(self.goal, self.calc_key(self.goal))
        
        self.processed_nodes = 0 #majd a metrikáknál használjuk fel, számlalóként használjuk, hogy mennyi csomópontot dolgozott fel az algoritmus
        
    def heuristic(self, node1, node2):
        # euklidészi távolságot számítunk, az átlós mozgás miatt
        return math.sqrt((node1[0] - node2[0])**2 + (node1[1] - node2[1])**2)
    
    
    def cost(self, node1, node2):
        # kiszámoljuk az irányt két pont között
        direction_y = abs(node1[0] - node2[0])
        diretcion_x = abs(node1[1] - node2[1])
        
        # ha az irány 2, tehát átlósan léünk akkor a költésg 1.41
        if direction_y + diretcion_x == 2:
            return math.sqrt(2)
        
        # ha az irány 1, nem átlósan lépünk, akkor a költség 1
        elif direction_y + diretcion_x == 1:
            return 1
    
    # prioritási kulcsot számolunk itt. A prioritás elsőnek k1 szerint dönt, majd k2 szerint.  
    def calc_key(self, node):
        # lehetséges legjobb út (vagy g vagy az rhs) + a heurisztika + a korrigáló változó
        k_1 = min(self.g[node], self.rhs[node]) + self.heuristic(node, self.start) + self.k_m
        # (vagy g vagy az rhs), amelyik kisebb
        k_2 = min(self.g[node], self.rhs[node])
        
        return (k_1, k_2)
    
    # beszúr az dictionaryba és prioritási sorba, ha javult a node, a régi bejegyzést pedig törli
    def insert(self, node, key):
        if node in self.open_dict:
            self.remove(node)
            
        heapq.heappush(self.U, (key, node))
        self.open_dict[node] = key
        
    def remove(self, node):
        if node in self.open_dict:
            del self.open_dict[node]
        
    
    # a csomópont körüli szomsédok meghatározása        
    def get_neighbors(self, node):
        neighbors = []
        
        # lehetséges irányok amerre mozoghatunk az aktuális csomópontból
        directions = [(-1, 0), (-1, 1), (0, 1), (1, 1), (1, 0), (1, -1), (0, -1), (-1, -1)]
         
        # végigmegy a ciklus az irányokon és az aktuális ponthoz, ahol állunk hozzádadja így meg kapjuk az új pont koordinátáis, ha odalépnénk    
        for direction_y, direction_x in directions:
            neighbor_y, neighbor_x = node[0] + direction_y, node[1] + direction_x
            
            # ellenőrizzük, hogy a griden belül vagyun-e és szabad-e a cella
            if 0 <= neighbor_y < len(self.grid) and 0 <= neighbor_x < len(self.grid[0]) and self.grid[neighbor_y][neighbor_x] == 0:
                         
                # átlós lépés miatt ellenőrzzük, hogy a szomszédos cellál, amik mellette vannak az adott irányból nem-e akadályok, ha az kihagyjuk  
                if abs(direction_y) > 0 and abs(direction_x) > 0:              
                    if self.grid[node[0]][neighbor_x] != 0 or self.grid[neighbor_y][node[1]] != 0:
                        continue
                    
                neighbors.append((neighbor_y, neighbor_x))
                
        return neighbors
    
    # szomszédok értékeinek frissítése    
    def update_vertex(self, node):
        
        # ha az aktuális node nem egyenlő a céllal, akkor kiszámoljuk a szomszédok g + odalépés költségeit és innen kiválasztjuk a legkisebbet, 
        # majd az aktuális node rhs értéket befrissítjük 
        if node != self.goal:
            self.rhs[node] = min([self.g[s] + self.cost(s, node) for s in self.get_neighbors(node)] or [math.inf])
        
        # ha változott a node g vagy rhs értéke, akkor eltávolítjuk az open_dict-ből, ezzel biztosítjuk, hogy új kulccsal tegyük vissza
        if node in self.open_dict:
            self.remove(node)
        
        # ha node inkonzisztensé válik, akkor újra felesszük a prioritási sorba (konzisztens, ha a g = rhs)
        if self.g[node] != self.rhs[node]:
            self.insert(node, self.calc_key(node))
     
    # az optimális út visszafejtése        
    def get_path(self):
        path = []
        current = self.start # a starttól indul a visszafejtés
        
        # ha a start node végtelen, akkor az algoritmus nem talált útvonalat
        if self.g[current] == math.inf:
            return []
        
        path.append(current) # betesszük az path listába, mint első elem
        
        # addig megy a ciklus, amíg el nem érjük a cél node-ot
        while current != self.goal:
            neighbors = self.get_neighbors(current) # az aktuális node-nek lekérjük a szomszédait
            neighbors = [n for n in neighbors if self.g[n] != math.inf] # kihagyjuk azokat, ahol a g értéke végtelen
            
            # ha nincs szomszéd, akkor út sincs
            if not neighbors:
                return []
            
            #mindig az lesz az aktuális a szomszédok közül, ahol a g + a lépési költség a legkisebb
            current = min(neighbors, key = lambda n: self.g[n] + self.cost(current, n))
            
            # majd ezt az elemet betesszük a listába
            path.append(current)
            
        return path
    
    def compute_shortest_path(self):
        # a ciklus addig fut, amíg az U-prioritási sor nem üres és a start prioritási kulcsa jobb, mint az U első eleme vagy a start rhs és g értéke konziszten nem lesz
        while self.U and (self.U[0][0] < self.calc_key(self.start) or self.rhs[self.start] != self.g[self.start]):
            # kivesszük a prioritáso sorból a legkisebb kulcsú elemet
            (k_old, node) = heapq.heappop(self.U)
            
            self.processed_nodes += 1 # csomópont feldolgozáshoz számlaló , ez a metrikához kell

            # itt megakadályozzuk a duplikált feldogozást. ha, a node nincs benne a dictionaryben vagy a régi kulcsa nem egyenlő az aktuális kulccsal, akkor kiahygjuk
            if node not in self.open_dict or k_old != self.open_dict[node]:
                continue
            #töröljük az elavult node-ot
            del self.open_dict[node]

            k_new = self.calc_key(node) # kiszámolunk egy új prioritási kulcsot az aktuális node-nak
            if k_old < k_new: # ha az új kulcs rosszabb, mint a korábbi, akkor visszetesszük az új kulccsel az open_dictbe. A node prioritása csökkent kevésbe fontos
                self.insert(node, k_new)

            # ha a g érétke javult, akkor frissítjük a szomszédokat
            elif self.g[node] > self.rhs[node]:
                self.g[node] = self.rhs[node]
                for n in self.get_neighbors(node):
                    self.update_vertex(n)

            else:
                # g értéke romlott lehet, hogy több szómszéd rhs-e is ezen a csúcson át volt optimális.
                g_old = self.g[node]
                self.g[node] = math.inf
                
                # ki kell terjeszteni a romlást a szomszédokra is, ha rajtuk keresztül ment az optimális út
                for n in self.get_neighbors(node):
                     # n optimális rhs értéke tényleg node-on keresztül megy, de mivel g végtelen lesz, így ez az útvonal érvénytelen
                    if self.rhs[n] == self.cost(n, node) + g_old:
                        # újraszámoljuk n rhs értékét, hogy megkeressük melyik most a legjobb szomszéd
                        self.rhs[n] = min([self.g[s] + self.cost(s, n) for s in self.get_neighbors(n)] or [math.inf])
                    self.update_vertex(n)
                    
                self.update_vertex(node)
                    
    def update_obstacle(self, node, is_obstacle):
        row_y, column_x = node

        # befrissítjük a rácsot az új akadállyal
        self.grid[row_y, column_x] = 1 if is_obstacle else 0 

        self.k_m += self.heuristic(self.start_last, self.start)
        self.start_last = self.start  

        # frissítjük az értintett csomópontot és a szomszédait 
        self.update_vertex(node)
        for n in self.get_neighbors(node):
            self.update_vertex(n)
