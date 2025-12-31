import numpy as np
import matplotlib.pyplot as plt
import math

class Node:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.cost = 0.0
        self.parent = None

class RRTStar:
    def __init__(self, terrain, start, goal, max_elevation_diff=5.0, expand_dis=3.0, path_resolution=1.0, goal_sample_rate=0.05, max_iter=500):
        self.terrain = terrain
        self.start = start
        self.goal = goal
        self.max_elevation_diff = max_elevation_diff
        self.expand_dis = expand_dis
        self.path_resolution = path_resolution
        self.goal_sample_rate = goal_sample_rate
        self.max_iter = max_iter
        self.node_list = [Node(start[0], start[1])]
        self.x_max, self.y_max = terrain.shape

    def distance(self, node1, node2):
        return math.sqrt((node1.x - node2.x) ** 2 + (node1.y - node2.y) ** 2)

    def steer(self, from_node, to_node, extend_length=float('inf')):
        """Steers from_node towards to_node"""
        new_node = Node(from_node.x, from_node.y)
        d = self.distance(from_node, to_node)
        if extend_length > d:
            extend_length = d

        theta = math.atan2(to_node.y - from_node.y, to_node.x - from_node.x)
        new_node.x += extend_length * math.cos(theta)
        new_node.y += extend_length * math.sin(theta)
        new_node.cost = from_node.cost + extend_length
        new_node.parent = from_node
        return new_node

    def get_nearest_node_index(self, rnd_node):
        dlist = [self.distance(node, rnd_node) for node in self.node_list]
        minind = dlist.index(min(dlist))
        return minind

    def get_neighborhood(self, new_node, radius):
        nnode = [node for node in self.node_list if self.distance(node, new_node) <= radius]
        return nnode

    def choose_parent(self, neighboring_nodes, new_node):
        if not neighboring_nodes:
            return None

        costs = []
        for node in neighboring_nodes:
            t_node = self.steer(node, new_node)
            if t_node:
                costs.append(t_node.cost)
            else:
                costs.append(float('inf'))

        min_cost = min(costs)
        min_index = costs.index(min_cost)

        if min_cost == float('inf'):
            return None

        new_node.cost = min_cost
        return neighboring_nodes[min_index]

    def rewire(self, new_node, neighboring_nodes):
        for node in neighboring_nodes:
            t_node = self.steer(new_node, node)
            if t_node and t_node.cost < node.cost:
                node.parent = new_node
                node.cost = t_node.cost

    def generate_random_node(self):
        if np.random.rand() > self.goal_sample_rate:
            return Node(np.random.uniform(0, self.x_max), np.random.uniform(0, self.y_max))
        else:
            return Node(self.goal[0], self.goal[1])

    def check_collision(self, node, parent_node):
        """Check if the node or path collides with obstacles in the terrain and elevation difference."""
        x, y = int(node.x), int(node.y)
        px, py = int(parent_node.x), int(parent_node.y)

        if x < 0 or x >= self.x_max or y < 0 or y >= self.y_max:
            return True  # Out of bounds

        if self.terrain[x, y] == -1:  # Obstacle in the terrain
            return True

        # Check if elevation difference is too large
        elevation_diff = abs(self.terrain[x, y] - self.terrain[px, py])
        if elevation_diff > self.max_elevation_diff:
            return True  # Elevation change is too steep

        return False

    def generate_final_course(self, goal_ind):
        path = [(self.node_list[goal_ind].x, self.node_list[goal_ind].y)]
        node = self.node_list[goal_ind]
        while node.parent is not None:
            node = node.parent
            path.append((node.x, node.y))
        path.reverse()
        return path

    def plot_path_2d(self, path):
        """Plot the terrain and the found path in 2D."""
        plt.imshow(self.terrain, cmap='terrain')
        path_x = [p[0] for p in path]
        path_y = [p[1] for p in path]

        # Plot the start, goal, and the path
        plt.plot(path_x, path_y, '-r', label="Path")
        plt.scatter(self.start[0], self.start[1], color='green', label='Start', zorder=5)
        plt.scatter(self.goal[0], self.goal[1], color='red', label='Goal', zorder=5)

        plt.legend()
        plt.title('2D Path Visualization - RRT*')
        plt.colorbar(label='Elevation')
        plt.show()

    def draw_graph(self, goal):
        plt.clf()
        for node in self.node_list:
            if node.parent:
                plt.plot([node.x, node.parent.x], [node.y, node.parent.y], "-g")

        plt.plot(goal.x, goal.y, "xr")
        plt.axis([0, self.x_max, 0, self.y_max])
        plt.grid(True)
        plt.pause(0.01)

    def plan(self):
        goal_node = Node(self.goal[0], self.goal[1])

        for i in range(self.max_iter):
            rnd_node = self.generate_random_node()
            nearest_ind = self.get_nearest_node_index(rnd_node)
            nearest_node = self.node_list[nearest_ind]

            new_node = self.steer(nearest_node, rnd_node, extend_length=self.expand_dis)
            if new_node and not self.check_collision(new_node, nearest_node):
                neighboring_nodes = self.get_neighborhood(new_node, radius=self.expand_dis * 2)
                new_node.parent = self.choose_parent(neighboring_nodes, new_node)
                if new_node.parent:
                    self.node_list.append(new_node)
                    self.rewire(new_node, neighboring_nodes)

            if self.distance(new_node, goal_node) <= self.expand_dis:
                final_node = self.steer(new_node, goal_node)
                if final_node and not self.check_collision(final_node, new_node):
                    return self.generate_final_course(len(self.node_list) - 1)

        return None  # Failed to find a path


'''
# Example Usage
terrain_with_obstacles = np.load('terrain_with_obstacles_and_points.npy')
start_point = (20, 30)
end_point = (180, 270)

# Initialize RRT* planner
rrt_star = RRTStar(terrain_with_obstacles, start_point, end_point)

# Run the planning algorithm
path = rrt_star.plan()

# If a path is found, visualize it
if path:
    print("Found path!")
    rrt_star.plot_path_2d(path)
else:
    print("No path found.")
'''