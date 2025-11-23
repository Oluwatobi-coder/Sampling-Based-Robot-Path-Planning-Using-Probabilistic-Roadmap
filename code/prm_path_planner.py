#!/usr/bin/env python3

# the command at the top ensures the code can be run on Linux or macOS using python3 interpreter

# importing the required modules
import csv # maniplation of csv files
import math # standard module for mathematical operations
import random # random number generation for sampling
import heapq # heap queue implementation for the OPEN list


# defining the node class
class Node:
    def __init__(self, id, x, y):
        self.id = id
        self.x = x
        self.y = y
        self.edges = [] # list of tuples i.e. (neighbor_node, cost)
        self.heuristic = 0.0 # the heuristic cost-to-go

    def add_edge(self, node, cost):
        self.edges.append((node, cost))

# defining the obstacle class
class Obstacle:
    def __init__(self, x, y, diameter):
        self.x = x
        self.y = y
        self.radius = diameter / 2.0

# function to load the obstacles from the obstacles.csv
def load_obstacles(filename):    
    obstacles = [] # initializing empty obstacles array
    try:
        with open(filename, 'r') as f:
            reader = csv.reader(f)
            
            for _ in range(5):
                next(reader)

            for row in reader:
                # Parsing x, y, diameter entries for each row
                obs = Obstacle(float(row[0]), float(row[1]), float(row[2]))
                obstacles.append(obs)
    except FileNotFoundError:
        print(f"Error: {filename} not found.")
    return obstacles

# function to read from the parameters.txt file and parse its contents
def load_parameters(filename):
    params = {
        'num_samples': 200,
        'k_neighbors': 10,
        'robot_radius': 0.0,
        'bounds_min': -0.5,
        'bounds_max': 0.5
    }
    try:
        with open(filename, 'r') as f:
            for line in f:
                if '=' in line:
                    key, value = line.strip().split('=')
                    if key in params:
                        if '.' in value:
                            params[key] = float(value)
                        else:
                            params[key] = int(value)
    except FileNotFoundError:
        print(f"Warning: {filename} not found. Using defaults.")
    return params






# function to calculate the Euclidean distance between any two nodes
def euclidean_dist(node1, node2):
    return math.sqrt((node1.x - node2.x)**2 + (node1.y - node2.y)**2)

# function to calculate the Euclidean distance between any two sets of coordinates
def distance_points(x1, y1, x2, y2):
    return math.sqrt((x1 - x2)**2 + (y1 - y2)**2)

# function to check if a line segment between 2 points i.e p1 and p2 intersects a circle 
# returns True if there is an intersection 
def check_intersection_line_circle(p1_x, p1_y, p2_x, p2_y, circle_x, circle_y, r):
    # Vector d = p2 - p1
    dx = p2_x - p1_x
    dy = p2_y - p1_y
    
    # Vector f = p1 - circle_center
    fx = p1_x - circle_x
    fy = p1_y - circle_y

    # Quadratic equation: a*t^2 + b*t + c = 0
    # intersection points are p(t) = p1 + t*d
    a = dx**2 + dy**2
    b = 2 * (fx*dx + fy*dy)
    c = (fx**2 + fy**2) - r**2

    discriminant = b**2 - 4*a*c

    if discriminant < 0:
        return False # no intersection
    else:
        # Check if the intersection points t1, t2 are within the segment [0, 1]
        discriminant = math.sqrt(discriminant)
        t1 = (-b - discriminant) / (2*a)
        t2 = (-b + discriminant) / (2*a)

        if (0 <= t1 <= 1) or (0 <= t2 <= 1):
            return True
        
        return False

# function for checking if the path between any two nodes is obstacle free 
# returns True if the path is collision free
def is_collision_free(n1, n2, obstacles, robot_radius):
    for obs in obstacles:
        # Effective radius = Obstacle Radius + Robot Radius
        effective_radius = obs.radius + robot_radius
        
        # checking if the endpoints exist inside circle
        if distance_points(n1.x, n1.y, obs.x, obs.y) <= effective_radius:
            return False 
        if distance_points(n2.x, n2.y, obs.x, obs.y) <= effective_radius:
            return False

        # checking for line segment intersection
        if check_intersection_line_circle(n1.x, n1.y, n2.x, n2.y, obs.x, obs.y, effective_radius):
            return False
            
    return True

# function for checking if a specific point lies inside any obstacle 
# returns True if the point lies outside the obstacle
def is_point_valid(x, y, obstacles, robot_radius):
    for obs in obstacles:
        if distance_points(x, y, obs.x, obs.y) <= (obs.radius + robot_radius):
            return False
    return True






# function to generate random samples i.e. sampling the  C-space
def generate_random_samples(params, obstacles):
    
    nodes = [] # intializing nodes array
    
    # defining the start and goal nodes
    start_node = Node(1, -0.5, -0.5)
    goal_node = Node(2, 0.5, 0.5)
    
    nodes.append(start_node)
    nodes.append(goal_node)
    
    node_counter = 3 # continues the node ID numbering from 3 since node 1 and 2 are defined already
    
    # generating the random samples and checking if each random points are valid
    while len(nodes) < params['num_samples']:
        rand_x = random.uniform(params['bounds_min'], params['bounds_max'])
        rand_y = random.uniform(params['bounds_min'], params['bounds_max'])
        
        if is_point_valid(rand_x, rand_y, obstacles, params['robot_radius']):
            new_node = Node(node_counter, rand_x, rand_y)
            nodes.append(new_node)
            node_counter += 1
            
    return nodes, start_node, goal_node

# function for building the edges i.e. graph
def build_graph(nodes, params, obstacles):
    print("Building the graph edges...")
    
    edges_list = [] # initializing the edges list
    
    for i, current_node in enumerate(nodes):
        # calculating the distance of a single node to all other nodes
        distances = []
        for other_node in nodes:
            if current_node.id == other_node.id:
                continue
            dist = euclidean_dist(current_node, other_node)
            distances.append((dist, other_node))
        
        # sorting by distance and picking k nearest neighbors
        distances.sort(key=lambda x: x[0])
        nearest_neighbors = distances[:params['k_neighbors']]
        
        # checking for collision for each neighbor edge and adding successful ones to the 
        # current node's edge and the list of edges
        for dist, neighbor in nearest_neighbors:
            if is_collision_free(current_node, neighbor, obstacles, params['robot_radius']):
                current_node.add_edge(neighbor, dist)
                edges_list.append((current_node.id, neighbor.id, dist))
                
    return edges_list





# function for determining the heuristic cost-to-go
def heuristic(node, goal_node):
    return euclidean_dist(node, goal_node)

# function for implementing the A* search algorithm
def a_star_search(start_node, goal_node, nodes):
    
    # setting up the priority queue i.e. OPEN list, to store tuples: (future_cost, current_node_id)
    open_set = []
    heapq.heappush(open_set, (start_node.id, 0))
    
    # mapping the ID to Node object for easy lookup
    id_to_node = {n.id: n for n in nodes}
    
    # initializing the past cost of the nodes to infinity
    p_cost = {n.id: float('inf') for n in nodes}
    # setting the past cost of node 1 to zero
    p_cost[start_node.id] = 0
    
    # initializing the future cost of the nodes to infinity
    f_cost = {n.id: float('inf') for n in nodes}
    # setting the future cost of node 1 to zero
    f_cost[start_node.id] = heuristic(start_node, goal_node)
    
    came_from = {} # dictionary for reconstructing the path i.e. tracking the parent nodes

    visited = set() # initializing the CLOSE set

    while open_set:
        # getting the node with lowest f_cost
        current_id, current_f_cost = heapq.heappop(open_set)
        current_node = id_to_node[current_id]
        
        # reconstructing the final path if the current node is the goal node
        if current_id == goal_node.id:
            return reconstruct_path(came_from, current_id)
        
        # skipping the current node if already evaluated
        if current_id in visited:
            continue
        
        # adding the current node to the CLOSE set has it is being evaluated
        visited.add(current_id)

        # exploring the neighbor nodes of the current node
        for neighbor, weight in current_node.edges:
            # calculating the speculative cost to reach the neighbor node
            speculative_cost = p_cost[current_id] + weight
            # checking if the speculative cost is lesser than the pass cost i.e. if this is 
            # a better path compared to previous
            if speculative_cost < p_cost[neighbor.id]:
                # adding the current node to the parent array
                came_from[neighbor.id] = current_id
                # storing the improved cost i.e. lower cost
                p_cost[neighbor.id] = speculative_cost
                f_cost[neighbor.id] = p_cost[neighbor.id] + heuristic(neighbor, goal_node)
                
                # adding the neighbor node to the OPEN list
                heapq.heappush(open_set, (neighbor.id, f_cost[neighbor.id]))

      # returning None if the loop finishes without finding the goal node i.e. no path exists           
    return None

# function for reconstructing the path once the goal node has been reached
def reconstruct_path(came_from, current_id):
    total_path = [current_id]
    while current_id in came_from:
        current_id = came_from[current_id]
        total_path.append(current_id)
    total_path.reverse() # reversing the list to begin from the start node to the goal node
    return total_path





# function for writing into the node.csv file
def write_nodes_file(filename, nodes, goal_node):
    with open(filename, 'w', newline='') as f:
        # ID, x, y, heuristic-cost-to-go as row entries
        for n in nodes:
            h_cost = heuristic(n, goal_node)
            f.write(f"{n.id},{n.x:.4f},{n.y:.4f},{h_cost:.4f}\n")

# function for writing into the edges.csv file
def write_edges_file(filename, edges):
    with open(filename, 'w', newline='') as f:
        # ID1, ID2, cost as row entries
        for e in edges:
            f.write(f"{e[0]},{e[1]},{e[2]:.4f}\n")

# function for writing into the path.csv file
def write_path_file(filename, path):
    with open(filename, 'w', newline='') as f:
        if path:
            # node1, node2, node3, ..., N as entries
            f.write(",".join(map(str, path)))
        else:
            f.write("No path found")

# defining the main function that calls the other functions when executed
def main():
    print("Starting PRM Planner...")
    
    # loading data from the input files, obstacles.csv and parameters.txt
    obstacles = load_obstacles('results/obstacles.csv')
    params = load_parameters('parameters/parameters.txt')
    
    print(f"Loaded {len(obstacles)} obstacles.")
    print(f"Params: {params}")
    
    # generating the sample nodes
    nodes, start_node, goal_node = generate_random_samples(params, obstacles)
    print(f"Generated {len(nodes)} samples.")
    
    # building the graph with the edges
    edges_data = build_graph(nodes, params, obstacles)
    print(f"Graph built with {len(edges_data)} edges.")
    
    # running the A* Search algorithm
    print("Running A* Search...")
    path = a_star_search(start_node, goal_node, nodes)
    
    # printing the path sequence once the search is successful
    if path:
        print("Path found successfully!")
        print(f"Path sequence: {path}")
    else:
        print("No path found.")
    
    # generating the output files: nodes.csv, edges.csv and path.csv
    write_nodes_file('results/nodes.csv', nodes, goal_node)
    write_edges_file('results/edges.csv', edges_data)
    write_path_file('results/path.csv', path)
    
    print("Output files generated: nodes.csv, edges.csv, path.csv")

# executing the main() function
if __name__ == "__main__":
    main()