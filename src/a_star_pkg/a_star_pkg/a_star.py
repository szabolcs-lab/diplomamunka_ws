import math
import heapq

class AStar:
    def __init__(self, grid, start, goal):
        self.grid = grid
        self.start = start
        self.goal = goal
        
        self.processed_nodes = 0

    def heuristic(self, a, b):
        return math.sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)

    def get_neighbors(self, node):
        neighbors = []
        directions = [(-1, 0), (-1, 1), (0, 1), (1, 1),
                      (1, 0), (1, -1), (0, -1), (-1, -1)]
        H, W = len(self.grid), len(self.grid[0])
        y, x = node
        for dy, dx in directions:
            ny, nx = y + dy, x + dx
            if 0 <= ny < H and 0 <= nx < W and self.grid[ny][nx] == 0:
                # sarokvágás tiltás
                if dy != 0 and dx != 0:
                    if self.grid[y][nx] != 0 or self.grid[ny][x] != 0:
                        continue
                neighbors.append((ny, nx))
        return neighbors

    def cost(self, a, b):
        dy = abs(a[0] - b[0])
        dx = abs(a[1] - b[1])
        if dy + dx == 2:
            return math.sqrt(2)
        elif dy + dx == 1:
            return 1
        else:
            return math.inf

    def reconstruct_path(self, came_from):
        current = self.goal
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        path.reverse()
        return path

    def a_star_plan(self):
        open_set = []
        heapq.heappush(open_set, (0 + self.heuristic(self.start, self.goal), 0, self.start))
        came_from = {}
        g_score = {self.start: 0}

        while open_set:
            _, current_g, current = heapq.heappop(open_set)
            
            self.processed_nodes += 1

            if current == self.goal:
                return self.reconstruct_path(came_from)

            for neighbor in self.get_neighbors(current):
                tentative_g = current_g + self.cost(current, neighbor)
                
                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    g_score[neighbor] = tentative_g
                    priority = tentative_g + self.heuristic(neighbor, self.goal)
                    heapq.heappush(open_set, (priority, tentative_g, neighbor))
                    came_from[neighbor] = current

        return []

