import math
from collections import defaultdict
import heapq


class DStarLite():
    def __init__(self, grid, start, goal):
        
        self.grid = grid
        self.start = start
        self.goal = goal
        
        self.g = defaultdict(lambda: math.inf)
        self.rhs = defaultdict(lambda: math.inf)
        
        self.g[self.goal] = math.inf
        self.rhs[self.goal] = 0.0
        
        self.k_m = 0.0
        
        self.U = []
        self.open_dict = {}
        self.insert(self.goal, self.calc_key(self.goal))
        
    def heuristic(self, node1, node2):
        return math.sqrt((node1[0] - node2[0])**2 + (node1[1] - node2[1])**2)
    
    def insert(self, node, key):
        if node in self.open_dict:
            self.remove(node)
            
        heapq.heappush(self.U, (key, node))
        self.open_dict[node] = key
            
    def remove(self, node):
        del self.open_dict[node]
        
    def cost(self, node1, node2):
        direction_y = abs(node1[0] - node2[0])
        direction_x = abs(node1[1] - node2[1])
        
        if direction_y + direction_x == 2:
            return math.sqrt(2)
        
        if direction_y + direction_x == 1:
            return 1
        
    def get_neighbors(self, node):
        neighbors = []
        
        directions = [(-1, 0), (-1, 1), (0, 1), (1, 1), (1, 0), (1, -1), (0, -1), (-1, -1)]
        
        for direction_y, direction_x in directions:
            neighbor_y, neighbor_x = node[0] + direction_y, node[1] + direction_x
            
            if 0 <= neighbor_y < len(self.grid) and 0 <= neighbor_x < len(self.grid[0]) and self.grid[neighbor_y][neighbor_x] == 0:
                
                if abs(direction_y) > 0 and abs(direction_x) > 0:
                    if self.grid[node[0]][direction_x] != 0 or  self.grid[direction_y][node[1]] != 0:
                        continue
                    
                neighbors.append((neighbor_y, neighbor_x))
                
        return neighbors
    
    def calc_key(self, node):
         
        k_1 = min(self.g[node], self.rhs[node]) + self.heuristic(node, self.start) + self.k_m
        k_2 = min(self.g[node], self.rhs[node])
        
        return (k_1, k_2)
    
    def update_vertex(self, node):
        
        if node != self.goal:
            self.rhs[node] = min([self.g[s] + self.cost(s, node) for s in self.get_neighbors(node)] or [math.inf])
        
        if node in self.open_dict:
            self.remove(node)
        
        if self.g[node] != self.rhs[node]:
            self.insert(node, self.calc_key(node))
            
    def get_path(self):
        path = []
        current = self.start
        
        if self.g[current] == math.inf:
            return []
        
        path.append(current) 
        
        while current != self.goal:
            neighbors = self.get_neighbors(current)
            neighbors = [n for n in neighbors if self.g[n] != math.inf]
        
            if not neighbors:
                return []
            
            current = min(neighbors, key = lambda n: self.g[n] + self.cost(current, n))
            
            
            path.append(current)
            
        return path
    
    def compute_shortest_path(self):
        while self.U and (self.U[0][0] < self.calc_key(self.start) or self.rhs[self.start] != self.g[self.start]):
            (k_old, node) = heapq.heappop(self.U)

            if node not in self.open_dict or k_old != self.open_dict[node]:
                continue
        
            del self.open_dict[node]

            k_new = self.calc_key(node)
            if k_old < k_new:
                self.insert(node, k_new)

            elif self.g[node] > self.rhs[node]:
                self.g[node] = self.rhs[node]
                for n in self.get_neighbors(node):
                    self.update_vertex(n)

            else:
                g_old = self.g[node]
                self.g[node] = math.inf
                
                for n in self.get_neighbors(node):
                    if self.rhs[n] == self.cost(n, node) + g_old:
                        self.rhs[n] = min([self.g[s] + self.cost(s, n) for s in self.get_neighbors(n)] or [math.inf])
                    self.update_vertex(n)
                    
                self.update_vertex(node)

    def update_obstacle(self, node, is_obstacle):
        row_y, column_x = node
        self.grid[row_y, column_x] = 1 if is_obstacle else 0 

        self.k_m += self.heuristic(self.start_last, self.start)
        self.start_last = self.start  

        self.update_vertex(node)
        for n in self.get_neighbors(node):
            self.update_vertex(n)
   
        
        
         