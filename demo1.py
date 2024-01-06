import json
import networkx as nx
from itertools import permutations
# Read input from JSON file
file_path=r'Student Handout\Input data\Soln\level0.json'
with open(file_path, 'r') as f:
    data=json.load(f)
#print(data)
G = nx.Graph()
for neighborhood in data['neighbourhoods'].keys():
    G.add_node(neighborhood)
for restaurant in data['restaurants'].keys():
    G.add_node(restaurant)
for neighborhood, values in data['neighbourhoods'].items():
    for i, distance in enumerate(values['distances']):
        if distance != 'INF':
            G.add_edge(neighborhood, f'n{i}', weight=distance)
for restaurant, values in data['restaurants'].items():
    for i, distance in enumerate(values['neighbourhood_distance']):
        if distance != 'INF':
            G.add_edge(restaurant, f'n{i}', weight=distance)

# Function to calculate the total distance of a path
def calculate_total_distance(path, G):
    total_distance = 0
    for i in range(len(path) - 1):
        total_distance += G[path[i]][path[i + 1]]['weight']
    return total_distance

# Nearest Neighbor Algorithm
def nearest_neighbor_algorithm(graph, start_node):
    path = [start_node]
    remaining_nodes = set(graph.nodes) - {start_node}

    while remaining_nodes:
        current_node = path[-1]
        nearest_neighbor = min(remaining_nodes, key=lambda node: graph[current_node][node]['weight'])
        path.append(nearest_neighbor)
        remaining_nodes.remove(nearest_neighbor)

    return path

# Find the shortest TSP path using Nearest Neighbor Algorithm
start_node = 'r0'
tsp_path = nearest_neighbor_algorithm(G, start_node)
tsp_path.append(start_node)  # Return to the starting node
tsp_distance = calculate_total_distance(tsp_path, G)

# Print the result
print("Shortest TSP Path:", tsp_path)
print("Shortest TSP Distance:", tsp_distance)