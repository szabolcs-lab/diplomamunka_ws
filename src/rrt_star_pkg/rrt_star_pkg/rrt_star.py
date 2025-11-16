import numpy as np
from math import sqrt, atan2, cos, sin


class Node:
    def __init__(self, y, x):
        self.y = y  # sorindex (függőleges)
        self.x = x  # oszlopindex (vízszintes)
        self.parent = None
        self.cost = 0.0


def node_distance(n1, n2):
    return sqrt((n1.x - n2.x)**2 + (n1.y - n2.y)**2)


class RRTStar:
    def __init__(self, grid, start, goal,
                 max_iter=2000, step_len=3,
                 goal_sample_rate=0.05, rewire_radius=6):
        self.grid = grid
        self.height, self.width = grid.shape
        self.start = Node(start[0], start[1])  # (y, x)
        self.goal = Node(goal[0], goal[1])      # (y, x)
        self.max_iter = max_iter
        self.step_len = step_len
        self.goal_sample_rate = goal_sample_rate
        self.rewire_radius = rewire_radius
        self.tree = []
        self.processed_nodes = 0

    # fő RRT* algoritmus
    def plan(self):
        self.tree = [self.start]

        for i in range(self.max_iter):
            rnd = self.sample()
            nearest_node = self.get_nearest(rnd)
            new_node = self.steer(nearest_node, rnd)

            if not self.is_free(new_node, nearest_node):
                continue

            new_node.parent = nearest_node
            new_node.cost = nearest_node.cost + node_distance(nearest_node, new_node)

            near_nodes = self.get_near_nodes(new_node)
            new_node = self.choose_parent(new_node, near_nodes)
            self.tree.append(new_node)
            self.rewire(new_node, near_nodes)
            
            self.processed_nodes += 1

            if node_distance(new_node, self.goal) < self.step_len:
                print(f"[RRT*] Goal reached at iteration {i}")
                return self.extract_path(new_node)

        print("[RRT*] Goal not reached within iterations.")
        return None

    # segédfüggvények
    def sample(self):
        """Véletlen minta (y,x) koordinátákban"""
        if np.random.rand() < self.goal_sample_rate:
            return self.goal
        return Node(np.random.uniform(0, self.height - 1),
                    np.random.uniform(0, self.width - 1))

    def get_nearest(self, rnd):
        return min(self.tree, key=lambda n: node_distance(n, rnd))

    def steer(self, from_node, to_node):
        d = node_distance(from_node, to_node)
        if d < 1e-6:
            return from_node
        theta = atan2(to_node.y - from_node.y, to_node.x - from_node.x)
        dist = min(self.step_len, d)
        new_y = from_node.y + dist * sin(theta)
        new_x = from_node.x + dist * cos(theta)
        return Node(new_y, new_x)

    def is_free(self, node, from_node=None):
        """Ellenőrzi, hogy a node (és a szülője felé vezető él) szabad-e"""
        gx = int(round(node.x))
        gy = int(round(node.y))
        if gx < 0 or gy < 0 or gx >= self.width or gy >= self.height:
            return False
        if self.grid[gy, gx] == 1:
            return False

        if from_node is None:
            return True

        steps = int(max(abs(node.x - from_node.x), abs(node.y - from_node.y)))
        for i in range(steps + 1):
            t = i / max(1, steps)
            x = int(round(from_node.x + t * (node.x - from_node.x)))
            y = int(round(from_node.y + t * (node.y - from_node.y)))
            if x < 0 or y < 0 or x >= self.width or y >= self.height:
                return False
            if self.grid[y, x] == 1:
                return False
        return True

    def get_near_nodes(self, new_node):
        return [n for n in self.tree if node_distance(n, new_node) <= self.rewire_radius]

    def choose_parent(self, new_node, near_nodes):
        if not near_nodes:
            return new_node

        min_cost = new_node.cost
        best_parent = new_node.parent

        for near in near_nodes:
            cost = near.cost + node_distance(near, new_node)
            if cost < min_cost and self.is_free(new_node, near):
                min_cost = cost
                best_parent = near

        new_node.parent = best_parent
        new_node.cost = min_cost
        return new_node

    def rewire(self, new_node, near_nodes):
        for near in near_nodes:
            cost_through_new = new_node.cost + node_distance(new_node, near)
            if cost_through_new < near.cost and self.is_free(new_node, near):
                near.parent = new_node
                near.cost = cost_through_new

    def extract_path(self, node):
        path = []
        while node is not None:
            path.append((node.y, node.x)) 
            node = node.parent
        path.reverse()
        return path
