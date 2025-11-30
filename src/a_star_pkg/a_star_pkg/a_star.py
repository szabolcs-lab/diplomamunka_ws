import math
import heapq

class AStar:
    """
    Klasszikus A* algoritmus, amely az útvonaétervezést végzi
    """
    def __init__(self, grid, start, goal):
        self.grid = grid
        self.start = start
        self.goal = goal    
        self.processed_nodes = 0 #majd a metrikáknál használjuk fel, számlalóként használjuk, hogy mennyi csomópontot dolgozott fel az algoritmus

    def heuristic(self, node1, node2):
        # euklidészi távolságot számítunk, az átlós mozgás miatt
        return math.sqrt((node1[0] - node2[0])**2 + (node1[1] - node2[1])**2)

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

    def reconstruct_path(self, came_from):
        path = []
        current = self.goal
        
        path.append(current)
        
        # key és value alapján összerakjuk a path-t, key a szülő ahonnan tovább léptünk az adott pontra
        while current in came_from:
            current = came_from[current]
            path.append(current)
            
        path.reverse() # visszafelé iratjuk ki, mert a cél ponttal kezdtük, így fogja visszaadni a helyes sorrendet a starttól
        
        return path

    def a_star_plan(self):
        open_list = []
        came_from = {}
        
        # beállítjuk a kezdő pozíció g_score = 0-ra, majd betesszük az f_score-ral( kezdő és cél közti eukliédzsi távolsággal) az open_list-be ami egy priority queue
        g_score = {self.start: 0}
        f_score =   g_score[self.start] + self.heuristic(self.start, self.goal) 
        heapq.heappush(open_list, (f_score, self.start))

        # addig megyünk ameddig van elem vagy nem értük el a célt
        while open_list:
            _, current = heapq.heappop(open_list) #kivesszük a queue-ból a legkisebb f_score értékű pontot
            
            current_g = g_score[current] # lekérjük dict-ből a hozzátartozó g_score-t
            
            self.processed_nodes += 1 # számlálás

            # megnézzük, hogy az aktuális pont egyenlő-e a céllal, ha igen visszaépítjük az útvonalat
            if current == self.goal:
                return self.reconstruct_path(came_from)

            # végigmegyünk a szomszédokon
            for neighbor in self.get_neighbors(current):
                tentative_g = current_g + self.cost(current, neighbor) # mindegyiknél kiszámoljuk a becsült g-t
                
                # megnézzük, hogy a új számolt becsült g_score kisebb-e, mint a korábbi
                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    g_score[neighbor] = tentative_g # ha igen akkor a becsült új g_score-t beállítjuk a szomszédnak
                    f_score = tentative_g + self.heuristic(neighbor, self.goal) # kiszámoljuk a becsült költséget
                    
                    # betsszük a neighbort a queue-be az f_score érétkével és hozzáadjuk a dictionary-hez a szülőjével együtt
                    heapq.heappush(open_list, (f_score, neighbor))
                    came_from[neighbor] = current

        return []

