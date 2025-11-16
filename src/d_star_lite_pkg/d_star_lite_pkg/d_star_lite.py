import math
from collections import defaultdict
import heapq

class DStarLite():
    def __init__(self, grid, start, goal):
        
        self.grid = grid
        
        self.start = start
        self.start_last = self.start
        self.goal = goal
        
        self.g = defaultdict(lambda: math.inf)
        self.rhs = defaultdict(lambda: math.inf)
        
        self.g[self.goal] = math.inf
        self.rhs[self.goal] = 0.0
        
        self.k_m = 0.0
        
        self.U = []
        self.open_dict = {}
        self.insert(self.goal, self.calc_key(self.goal))
        
        self.processed_nodes = 0
        
    def heuristic(self, a, b):
        return math.sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)
    
    def cost(self, a, b):
        dx = abs(a[0] - b[0])
        dy = abs(a[1] - b[1])
        
        if dx + dy == 2:
            return math.sqrt(2)
        elif dx + dy == 1:
            return 1
        else:
            return math.inf
        
    def calc_key(self, node):
        k_1 = min(self.g[node], self.rhs[node]) + self.heuristic(node, self.start) + self.k_m
        k_2 = min(self.g[node], self.rhs[node])
        
        return (k_1, k_2)
    
    def insert(self, node, key):
        if node in self.open_dict:
            self.remove(node)
            
        heapq.heappush(self.U, (key, node))
        self.open_dict[node] = key
        
    def remove(self, node):
        if node in self.open_dict:
            del self.open_dict[node]
    '''       
    def get_neighbors(self, grid, u):
        neighbors = []
        
        direction = [(-1, 0), (-1, 1), (0, 1), (1, 1), (1, 0), (1, -1), (0, -1), (-1, -1)]
        
        for dx, dy in direction:
            neighbor = (u[0] + dx, u[1] + dy)
            if 0 <= neighbor[0] < len(grid) and 0 <= neighbor[1] < len(grid[0]):
                if grid[neighbor[0]][neighbor[1]] == 0:
                    neighbors.append(neighbor)
                    
        return neighbors
    '''
    
    def get_neighbors(self, grid, u):      
        neighbors = []
        H, W = grid.shape
        dirs = [(-1, 0), (-1, 1), (0, 1), (1, 1),
                (1, 0), (1, -1), (0, -1), (-1, -1)]

        uy, ux = u
        for dy, dx in dirs:
            ny, nx = uy + dy, ux + dx
            # rácson belül?
            if not (0 <= ny < H and 0 <= nx < W):
                continue
            # célcella szabad?
            if grid[ny, nx] != 0:
                continue
            # NO-CORNER-CUT: diagonálnál mindkét „oldalcellának” is szabadnak kell lennie
            if dy != 0 and dx != 0:
                if grid[uy, nx] != 0 or grid[ny, ux] != 0:
                    continue
            neighbors.append((ny, nx))
        return neighbors

    def update_vertex(self, u):
        if u != self.goal:
            self.rhs[u] = min([self.g[s] + self.cost(s, u) for s in self.get_neighbors(self.grid, u)] or [math.inf])
        
        if u in self.open_dict:
            self.remove(u)
        
        if self.g[u] != self.rhs[u]:
            self.insert(u, self.calc_key(u))
            
    def get_path(self):
        path = []
        current = self.start
        
        if self.g[current] == math.inf:
            return []
        
        path.append(current)
        
        while current != self.goal:
            neighbors = self.get_neighbors(self.grid, current)
            neighbors = [n for n in neighbors if self.g[n] != math.inf]
            
            if not neighbors:
                return []
            
            current = min(neighbors, key = lambda n: self.g[n] + self.cost(current, n))
            
            path.append(current)
            
        return path
    
    def compute_shortest_path(self):
        while self.U and (self.U[0][0] < self.calc_key(self.start) or self.rhs[self.start] != self.g[self.start]):
            (k_old, u) = heapq.heappop(self.U)
            
            self.processed_nodes += 1

            # stale bejegyzés kihagyása
            if u not in self.open_dict or k_old != self.open_dict[u]:
                continue
            del self.open_dict[u]

            k_new = self.calc_key(u)
            if k_old < k_new:
                self.insert(u, k_new)

            elif self.g[u] > self.rhs[u]:
                # g javult -> PRO-PAGÁLJ a szomszédoknak
                self.g[u] = self.rhs[u]
                for n in self.get_neighbors(self.grid, u):
                    self.update_vertex(n)

            else:
                # g romlott -> lehet, hogy több szómszéd rhs-e is ezen a csúcson át volt optimális
                g_old = self.g[u]
                self.g[u] = math.inf
                for n in self.get_neighbors(self.grid, u) + [u]:
                    if self.rhs[n] == self.cost(n, u) + g_old:
                        self.rhs[n] = min(
                            [self.g[s] + self.cost(s, n) for s in self.get_neighbors(self.grid, n)] or [math.inf]
                        )
                    self.update_vertex(n)
                    
    def update_obstacle(self, node, is_obstacle):
        x, y = node
        self.grid[x][y] = 1 if is_obstacle else 0 
        
        self.k_m += self.heuristic(self.start_last, self.start)
        self.start_last = self.start  
        
        self.update_vertex(node)
        for n in self.get_neighbors(self.grid, node):
            self.update_vertex(n)