import numpy as np
import matplotlib.pyplot as plt
import math

class Node:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.cost = 0.0
        self.parent = None

def distance(node1, node2):
    return math.sqrt((node1.x - node2.x) ** 2 + (node1.y - node2.y) ** 2)

def steer(from_node, to_node, extend_length=float('inf')):
    """Steers from_node towards to_node"""
    new_node = Node(from_node.x, from_node.y)
    d = distance(from_node, to_node)
    if extend_length > d:
        extend_length = d

    theta = math.atan2(to_node.y - from_node.y, to_node.x - from_node.x)
    new_node.x += extend_length * math.cos(theta)
    new_node.y += extend_length * math.sin(theta)
    new_node.cost = from_node.cost + extend_length
    new_node.parent = from_node
    return new_node

def get_nearest_node_index(node_list, rnd_node):
    dlist = [distance(node, rnd_node) for node in node_list]
    minind = dlist.index(min(dlist))
    return minind

def get_neighborhood(node_list, new_node, radius):
    nnode = [node for node in node_list if distance(node, new_node) <= radius]
    return nnode

def choose_parent(neighboring_nodes, new_node):
    if not neighboring_nodes:
        return None

    costs = []
    for node in neighboring_nodes:
        t_node = steer(node, new_node)
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

def rewire(node_list, new_node, neighboring_nodes):
    for node in neighboring_nodes:
        t_node = steer(new_node, node)
        if t_node and t_node.cost < node.cost:
            node.parent = new_node
            node.cost = t_node.cost

def generate_random_node(x_max, y_max, goal, goal_sample_rate):
    if np.random.rand() > goal_sample_rate:
        return Node(np.random.uniform(0, x_max), np.random.uniform(0, y_max))
    else:
        return goal

def draw_graph(node_list, goal):
    plt.clf()
    for node in node_list:
        if node.parent:
            plt.plot([node.x, node.parent.x], [node.y, node.parent.y], "-g")

    plt.plot(goal.x, goal.y, "xr")
    plt.axis([0, 100, 0, 100])
    plt.grid(True)
    plt.pause(0.01)

def rrt_star_planning(start, goal, obstacle_list, x_max, y_max, expand_dis=3.0, path_resolution=1.0, goal_sample_rate=0.05, max_iter=500):
    start_node = Node(start[0], start[1])
    goal_node = Node(goal[0], goal[1])
    node_list = [start_node]

    for i in range(max_iter):
        rnd_node = generate_random_node(x_max, y_max, goal_node, goal_sample_rate)
        nearest_ind = get_nearest_node_index(node_list, rnd_node)
        nearest_node = node_list[nearest_ind]

        new_node = steer(nearest_node, rnd_node, extend_length=expand_dis)
        if new_node and not check_collision(new_node, obstacle_list, path_resolution):
            neighboring_nodes = get_neighborhood(node_list, new_node, radius=expand_dis*2)
            new_node.parent = choose_parent(neighboring_nodes, new_node)
            if new_node.parent:
                node_list.append(new_node)
                rewire(node_list, new_node, neighboring_nodes)

        if distance(new_node, goal_node) <= expand_dis:
            final_node = steer(new_node, goal_node)
            if final_node and not check_collision(final_node, obstacle_list, path_resolution):
                return generate_final_course(len(node_list) - 1, node_list, start_node)

        draw_graph(node_list, goal_node)

    return None  # Failed to find a path

def check_collision(node, obstacle_list, path_resolution):
    # This function should check if the path between node and its parent intersects any obstacle
    # Placeholder for collision checking code
    return False

def generate_final_course(goal_ind, node_list, start_node):
    path = [(goal_ind.x, goal_ind.y)]
    node = node_list[goal_ind]
    while node.parent is not start_node:
        node = node.parent
        path.append((node.x, node.y))
    path.append((start_node.x, start_node.y))
    return path

# # Parameters
# start = (0, 0)
# goal = (100, 100)
# x_max = 100
# y_max = 100
# obstacle_list = []

# # Main
# path = rrt_star_planning(start, goal, obstacle_list, x_max, y_max)
# if path is not None:
   
