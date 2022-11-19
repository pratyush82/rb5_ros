from math import inf
import matplotlib
from matplotlib import pyplot as plt

# Initializing global variables
start_point = (0, 0)
end_point = (1, 1)
obs = (0.5, 0.5)
obs_width = obs_length = 0.33+0.10

class Square():
    def __init__(self, center_x, center_y, width, length) -> None:
        self.center_x = center_x
        self.center_y = center_y
        self.width = width
        self.length = length
        self.status = self.checkStatus()
        self.children = []

    def checkStatus(self):
        if (obs[0] - obs_width/2 <= self.center_x - self.width/2 <= self.center_x + self.width/2 <= obs[0] + obs_width/2 and
                obs[1] - obs_length/2 <= self.center_y - self.length/2 <= self.center_y + self.length/2 < obs[1] + obs_length/2):
            return 'obs'
        elif (self.center_x - self.width/2 < obs[0] + obs_width/2 and self.center_y - self.length/2 < obs[1] + obs_length/2 and
              self.center_x + self.width/2 > obs[0] - obs_width/2 and self.center_y + self.length/2 > obs[1] - obs_length/2):
            return 'mixed'
        else:
            return 'free'

class Node:
    def __init__(self, value, cordinates, neighbors=None):
        self.value = value
        self.x = cordinates[0]
        self.y = cordinates[1]
        self.heuristic_value = -1
        self.distance_from_start = inf
        if neighbors is None:
            self.neighbors = []
        else:
            self.neighbors = neighbors
        self.parent = None

    def has_neighbors(self):
        if len(self.neighbors) == 0:
            return False
        return True

    def number_of_neighbors(self):
        return len(self.neighbors)

    def add_neighboor(self, neighboor):
        self.neighbors.append(neighboor)

    def extend_node(self):
        children = []
        for child in self.neighbors:
            children.append(child[0])
        return children

    def __gt__(self, other):
        if isinstance(other, Node):
            if self.heuristic_value > other.heuristic_value:
                return True
            if self.heuristic_value < other.heuristic_value:
                return False
            return self.value > other.value

    def __eq__(self, other):
        if isinstance(other, Node):
            return self.value == other.value
        return self.value == other

    def __str__(self):
        return self.value

class Graph:
    def __init__(self, nodes=None):
        if nodes is None:
            self.nodes = []
        else:
            self.nodes = nodes

    def add_node(self, node):
        self.nodes.append(node)

    def find_node(self, value):
        for node in self.nodes:
            if node.value == value:
                return node
        return None

    def add_edge(self, value1, value2, weight=1):
        node1 = self.find_node(value1)
        node2 = self.find_node(value2)
        if (node1 is not None) and (node2 is not None):
            node1.add_neighboor((node2, weight))
            node2.add_neighboor((node1, weight))
        else:
            print("Error: One or more nodes were not found")

    def number_of_nodes(self):
        return f"The graph has {len(self.nodes)} nodes"

    def are_connected(self, node_one, node_two):
        node_one = self.find_node(node_one)
        node_two = self.find_node(node_two)
        for neighboor in node_one.neighbors:
            if neighboor[0].value == node_two.value:
                return True
        return False

    def __str__(self):
        graph = ""
        for node in self.nodes:
            graph += f"{node.__str__()}\n"
        return graph

class AStar:
    def __init__(self, graph, start_position, target):
        self.graph = graph
        self.start = graph.find_node(start_position)
        self.target = graph.find_node(target)
        self.opened = []
        self.closed = []
        self.number_of_steps = 0

    def manhattan_distance(self, node1, node2):
        return abs(node1.x - node2.x) + abs(node1.y - node2.y)

    def calculate_distance(self, parent, child):
        for neighbor in parent.neighbors:
            if neighbor[0] == child:
                distance = parent.distance_from_start + neighbor[1]
                if distance < child.distance_from_start:
                    child.parent = parent
                    return distance
                return child.distance_from_start

    def calculate_heuristic_value(self, parent, child, target):
        return self.calculate_distance(parent, child) + self.manhattan_distance(child, target)

    def insert_to_list(self, list_category, node):
        if list_category == "open":
            self.opened.append(node)
        else:
            self.closed.append(node)

    def remove_from_opened(self):
        self.opened.sort()
        node = self.opened.pop(0)
        self.closed.append(node)
        return node

    def opened_is_empty(self):
        return len(self.opened) == 0

    def get_old_node(self, node_value):
        for node in self.opened:
            if node.value == node_value:
                return node
        return None

    def calculate_path(self, target_node):
        path = [target_node.value]
        node = target_node.parent
        while True:
            path.append(node.value)
            if node.parent is None:
                break
            node = node.parent
        return path

    def calculate_cost(self, path):
        total_cost = 0
        for i in range(len(path) - 1):
            child = self.graph.find_node(path[i])
            parent = self.graph.find_node(path[i+1])
            for neighbor in child.neighbors:
                # Structure of neighbor(Node, weight)
                if neighbor[0] == parent:
                    total_cost += neighbor[1]
        return total_cost

    def search(self):
        # Calculate the heuristic value of the starting node
        # The distance from the starting node is 0 so only manhattan_distance is calculated
        self.start.distance_from_start = 0
        self.start.heuristic_value = self.manhattan_distance(self.start, self.target)
        # Add the starting point to opened list
        self.opened.append(self.start)
        while True:
            self.number_of_steps += 1
            if self.opened_is_empty():
                print(f"No Solution Found after {self.number_of_steps} steps!!!")
                return None, None
            selected_node = self.remove_from_opened()
            # check if the selected_node is the solution
            if selected_node == self.target:
                path = self.calculate_path(selected_node)
                total_cost = self.calculate_cost(path)
                path.reverse()
                return path, total_cost
            # extend the node
            new_nodes = selected_node.extend_node()
            # add the extended nodes in the list opened
            if len(new_nodes) > 0:
                for new_node in new_nodes:
                    new_node.heuristic_value = self.calculate_heuristic_value(
                        selected_node, new_node, self.target)
                    if new_node not in self.closed and new_node not in self.opened:
                        new_node.parent = selected_node
                        self.insert_to_list("open", new_node)
                    elif new_node in self.opened and new_node.parent != selected_node:
                        old_node = self.get_old_node(new_node.value)
                        if new_node.heuristic_value < old_node.heuristic_value:
                            new_node.parent = selected_node
                            self.insert_to_opened(new_node)

def intersect(square1, square2):
    return (square1.center_x - square1.width/2 <= square2.center_x + square2.width/2 and square1.center_y - square1.length/2 <= square2.center_y + square2.length/2 and
            square1.center_x + square1.width/2 >= square2.center_x - square2.width/2 and square1.center_y + square1.length/2 >= square2.center_y - square2.length/2)

def makeGraph(freenodes):
    adjacency_dict = {}
    for node in freenodes:
        for node2 in freenodes:
            if intersect(node, node2):
                node_val = (node.center_x, node.center_y)
                node2_val = (node2.center_x, node2.center_y)
                if node_val == node2_val:
                    continue
                if node_val in adjacency_dict:
                    adjacency_dict[node_val] = adjacency_dict[node_val] + [node2_val]
                else:
                    adjacency_dict[node_val] = [node2_val]
    return adjacency_dict

def aStarWrapper(a):
    import math
    graph = Graph()
    dict_name_mapping = {}
    rev_dict_name_mapping = {}
    for i in range(len(a.keys())):
        graph.add_node(Node(str(i), list(a.keys())[i]))
        dict_name_mapping[list(a.keys())[i]] = str(i)
        rev_dict_name_mapping[str(i)] = list(a.keys())[i]
    for i in range(len(a.keys())):
        for el in a[list(a.keys())[i]]:
            graph.add_edge(dict_name_mapping[list(a.keys())[i]], dict_name_mapping[el], math.dist(list(a.keys())[i], el))
            graph.add_edge(dict_name_mapping[el], dict_name_mapping[list(a.keys())[i]], math.dist(list(a.keys())[i], el))
    alg = AStar(graph, '0', '1')
    path, path_length = alg.search()
    if path is None:
        return []
    else:
        print(" -> ".join([str(rev_dict_name_mapping[el]) for el in path]))
        print(f"Length of the path: {path_length}")
        return [rev_dict_name_mapping[el] for el in path]

def findSafestPath(start_point, end_point):
    fig, ax = plt.subplots(1, 1, figsize=(10, 10))
    ax.add_patch(matplotlib.patches.Rectangle((obs[0] - obs_width/2, obs[1] - obs_length/2), obs_width, obs_length, facecolor='black', edgecolor='black', alpha=0.2, lw=2))
    col_dict = {'mixed': 'orange', 'obs': 'red', 'free': 'blue'}
    start_square = Square(start_point[0], start_point[1], 1e-3, 1e-3)
    end_square = Square(end_point[0], end_point[1], 1e-3, 1e-3)
    free_nodes = [start_square, end_square]
    root_square = Square(0.5, 0.5, 1, 1)
    queue = []
    queue.append(root_square)
    while queue:
        square = queue.pop(0)
        colr = col_dict[square.status]
        ax.add_patch(matplotlib.patches.Rectangle((square.center_x - square.width/2, square.center_y - square.length/2), square.width, square.length, edgecolor=colr, fill=False))
        ad_dict = {}
        if square.status == 'free':
            free_nodes.append(square)
            ad_dict = makeGraph(free_nodes)
            if (start_point[0], start_point[1]) in list(ad_dict.keys()) and (end_point[0], end_point[1]) in list(ad_dict.keys()):
                found_path = aStarWrapper(ad_dict)
                if len(found_path) != 0:
                    break
        if square.status == 'mixed':
            NW = Square(square.center_x - square.width/4, square.center_y + square.length/4, square.width/2, square.length/2)
            NE = Square(square.center_x + square.width/4, square.center_y + square.length/4, square.width/2, square.length/2)
            SW = Square(square.center_x - square.width/4, square.center_y - square.length/4, square.width/2, square.length/2)
            SE = Square(square.center_x + square.width/4, square.center_y - square.length/4, square.width/2, square.length/2)
            queue.append(NW)
            queue.append(NE)
            queue.append(SW)
            queue.append(SE)
            square.children = [NW, NE, SW, SE]
    ax.plot([el[0] for el in found_path], [el[1] for el in found_path], '-.go')
    for el in found_path:
        ax.text(el[0]+2e-2, el[1]+2e-2, str(el), fontsize=10, fontweight='bold')
    ax.set_xlim((-0.2, 1.2))
    ax.set_ylim((-0.2, 1.2))
    plt.show()
    return found_path

# Main function
final_waypoints = findSafestPath(start_point, end_point)
