import numpy as np
import heapq
import matplotlib.pyplot as plt
import math

class Node:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.cost = float('inf')  # Initial cost is set to infinity
        self.parent = None

class Dijkstra:
    def __init__(self, terrain, start, goal, max_elevation_diff=5.0):
        self.terrain = terrain
        self.start = start
        self.goal = goal
        self.max_elevation_diff = max_elevation_diff
        self.x_max, self.y_max = terrain.shape

        # Create a grid of nodes
        self.node_grid = [[Node(x, y) for y in range(self.y_max)] for x in range(self.x_max)]
        self.node_grid[start[0]][start[1]].cost = 0  # Start node has 0 cost

    def distance(self, node1, node2):
        return math.sqrt((node1.x - node2.x) ** 2 + (node1.y - node2.y) ** 2)

    def get_neighbors(self, node):
        neighbors = []
        for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
            nx, ny = node.x + dx, node.y + dy
            if 0 <= nx < self.x_max and 0 <= ny < self.y_max and self.terrain[nx, ny] != -1:
                neighbors.append(self.node_grid[nx][ny])
        return neighbors

    def check_elevation_difference(self, node, neighbor):
        elevation_diff = abs(self.terrain[node.x, node.y] - self.terrain[neighbor.x, neighbor.y])
        return elevation_diff <= self.max_elevation_diff

    def plan(self):
        start_node = self.node_grid[self.start[0]][self.start[1]]
        goal_node = self.node_grid[self.goal[0]][self.goal[1]]

        # Priority queue to hold the nodes to be explored
        open_list = [(0, start_node)]
        heapq.heapify(open_list)

        while open_list:
            current_cost, current_node = heapq.heappop(open_list)

            # Early exit if we reach the goal
            if current_node == goal_node:
                return self.generate_final_course(goal_node)

            # Explore neighbors
            for neighbor in self.get_neighbors(current_node):
                if self.check_elevation_difference(current_node, neighbor):
                    new_cost = current_cost + self.distance(current_node, neighbor)
                    if new_cost < neighbor.cost:
                        neighbor.cost = new_cost
                        neighbor.parent = current_node
                        heapq.heappush(open_list, (new_cost, neighbor))

        return None  # No path found

    def generate_final_course(self, goal_node):
        path = [(goal_node.x, goal_node.y)]
        node = goal_node
        while node.parent is not None:
            node = node.parent
            path.append((node.x, node.y))
        path.reverse()
        return path

    def plot_path_2d(self, path, save_path=None):
        """Plot the terrain and the found path in 2D, and optionally save it as a PNG file."""
        plt.imshow(self.terrain, cmap='terrain')
        path_x = [p[0] for p in path]
        path_y = [p[1] for p in path]

        # Plot the start, goal, and the path
        plt.plot(path_x, path_y, '-r', label="Path")
        plt.scatter(self.start[0], self.start[1], color='green', label='Start', zorder=5)
        plt.scatter(self.goal[0], self.goal[1], color='red', label='Goal', zorder=5)

        plt.legend()
        plt.title('2D Path Visualization')
        plt.colorbar(label='Elevation')

        if save_path:
            plt.savefig(save_path)  # Save the plot as a PNG file
            print(f"Path plot saved as {save_path}")
        else:
            plt.show()

        plt.close()



'''
# Example Usage
terrain_with_obstacles = np.load('terrain_with_obstacles_and_points.npy')
start_point = (20, 30)
end_point = (180, 270)

# Initialize Dijkstra planner
dijkstra = Dijkstra(terrain_with_obstacles, start_point, end_point)

# Run the planning algorithm
path = dijkstra.plan()

# If a path is found, visualize and optionally save it as a PNG
if path:
    print("Found path!")
    dijkstra.plot_path_2d(path, save_path='dijkstra_path.png')
else:
    print("No path found.")

'''