import numpy as np
import heapq

class Node:
    def __init__(self, x, y, cost=0.0, heuristic=0.0):
        self.x = x
        self.y = y
        self.cost = cost
        self.heuristic = heuristic
        self.total_cost = self.cost + self.heuristic
        self.parent = None

    def __lt__(self, other):
        return self.total_cost < other.total_cost

def heuristic(a, b):
    return np.hypot(b[0] - a[0], b[1] - a[1])

def a_star_search(start, goal, obstacles, grid_size, movement='4way'):
    open_set = []
    heapq.heappush(open_set, Node(start[0], start[1], 0.0, heuristic(start, goal)))
    closed_set = set()
    movements = [(-1, 0), (1, 0), (0, -1), (0, 1)] if movement == '4way' else [(-1, 0), (1, 0), (0, -1), (0, 1), (-1, -1), (-1, 1), (1, -1), (1, 1)]

    while open_set:
        current_node = heapq.heappop(open_set)

        if (current_node.x, current_node.y) in closed_set:
            continue

        if (current_node.x, current_node.y) == goal:
            path = []
            while current_node.parent is not None:
                path.append((current_node.x, current_node.y))
                current_node = current_node.parent
            path.append((start[0], start[1]))
            return path[::-1]

        closed_set.add((current_node.x, current_node.y))

        for move in movements:
            next_x, next_y = current_node.x + move[0], current_node.y + move[1]

            if 0 <= next_x < grid_size[0] and 0 <= next_y < grid_size[1] and (next_x, next_y) not in obstacles:
                next_node = Node(next_x, next_y, current_node.cost + np.hypot(move[0], move[1]), heuristic((next_x, next_y), goal))
                next_node.parent = current_node
                heapq.heappush(open_set, next_node)

    return None  # Path not found

# Example usage:
start = (0, 0)
goal = (7, 7)
obstacles = {(1, 2), (2, 2), (3, 2)}
grid_size = (8, 8)

path = a_star_search(start, goal, obstacles, grid_size, movement='4way')
print("Path found by A*:", path)
