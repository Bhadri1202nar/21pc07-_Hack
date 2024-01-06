import networkx as nx
import json

def create_graph(input_data):
    G = nx.Graph()
    neighbourhoods = input_data["neighbourhoods"]
    restaurants = input_data["restaurants"]
    vehicles = input_data["vehicles"]

    # Add neighbourhood nodes
    for n, data in neighbourhoods.items():
        G.add_node(n, demand=data["order_quantity"])

    # Add restaurant node
    for r, data in restaurants.items():
        G.add_node(r, demand=-sum(neighbourhoods[n]["order_quantity"] for n in neighbourhoods))

    # Add edges with distances
    for n, data_n in neighbourhoods.items():
        for r, distance in zip(restaurants.keys(), data_n["distances"]):
            G.add_edge(n, r, weight=distance)

    return G

def solve_cvrp(G, vehicle_capacity, num_paths, target_vehicle):
    vehicles = input_data["vehicles"]
    results = {}

    for v, data_v in vehicles.items():
        if v != target_vehicle:
            continue  # Skip vehicles other than the target vehicle

        depot = data_v["start_point"]
        speed = data_v["speed"]
        capacity = data_v["capacity"]

        # Solve the CVRP using NetworkX
        routes = nx.approximation.traveling_salesman_problem(G, weight='weight', cycle=True)

        # Organize the results, keeping only the desired number of paths
        paths = {"path{}".format(i + 1): route for i, route in enumerate(routes[:num_paths])}
        results[v] = paths

    return results

if __name__ == '__main__':
    input_data = { "n_neighbourhoods": 20,
  "n_restaurants": 1,
  "neighbourhoods": {
    "n0": {
      "order_quantity": 70,
      "distances": [
        0,
        2953,
        1170,
        1677,
        1318,
        2055,
        591,
        3050,
        2626,
        1864,
        277,
        2499,
        769,
        1463,
        2006,
        2516,
        2394,
        997,
        1099,
        421
      ]
    },
    "n1": {
      "order_quantity": 70,
      "distances": [
        2953,
        0,
        1783,
        1276,
        1635,
        898,
        2458,
        97,
        423,
        1089,
        3026,
        664,
        2280,
        1600,
        1057,
        535,
        559,
        2182,
        2208,
        2532
      ]
    },
    "n2": {
      "order_quantity": 90,
      "distances": [
        1170,
        1783,
        0,
        507,
        148,
        885,
        675,
        1880,
        1456,
        694,
        1447,
        1697,
        497,
        2633,
        2090,
        1346,
        1224,
        2167,
        2269,
        953
      ]
    },
    "n3": {
      "order_quantity": 50,
      "distances": [
        1677,
        1276,
        507,
        0,
        359,
        752,
        1182,
        1373,
        1325,
        187,
        1750,
        1566,
        1004,
        2502,
        1959,
        839,
        717,
        2036,
        2138,
        1256
      ]
    },
    "n4": {
      "order_quantity": 70,
      "distances": [
        1318,
        1635,
        148,
        359,
        0,
        737,
        823,
        1732,
        1310,
        546,
        1391,
        1551,
        645,
        2487,
        1944,
        1198,
        1076,
        2021,
        2123,
        897
      ]
    },
    "n5": {
      "order_quantity": 90,
      "distances": [
        2055,
        898,
        885,
        752,
        737,
        0,
        1560,
        995,
        573,
        939,
        2128,
        814,
        1382,
        1750,
        1207,
        461,
        339,
        1284,
        1386,
        1634
      ]
    },
    "n6": {
      "order_quantity": 110,
      "distances": [
        591,
        2458,
        675,
        1182,
        823,
        1560,
        0,
        2555,
        2131,
        1369,
        868,
        2004,
        178,
        2054,
        1511,
        2021,
        1899,
        1588,
        1690,
        374
      ]
    },
    "n7": {
      "order_quantity": 70,
      "distances": [
        3050,
        97,
        1880,
        1373,
        1732,
        995,
        2555,
        0,
        424,
        1186,
        3123,
        665,
        2377,
        1601,
        1058,
        534,
        656,
        2279,
        2305,
        2629
      ]
    },
    "n8": {
      "order_quantity": 110,
      "distances": [
        2626,
        423,
        1456,
        1325,
        1310,
        573,
        2131,
        424,
        0,
        1512,
        2699,
        241,
        1953,
        1177,
        634,
        958,
        608,
        1855,
        1881,
        2205
      ]
    },
    "n9": {
      "order_quantity": 70,
      "distances": [
        1864,
        1089,
        694,
        187,
        546,
        939,
        1369,
        1186,
        1512,
        0,
        1937,
        1753,
        1191,
        2689,
        2146,
        652,
        904,
        2223,
        2325,
        1443
      ]
    },
    "n10": {
      "order_quantity": 70,
      "distances": [
        277,
        3026,
        1447,
        1750,
        1391,
        2128,
        868,
        3123,
        2699,
        1937,
        0,
        2572,
        1046,
        1536,
        2079,
        2589,
        2467,
        844,
        822,
        494
      ]
    },
    "n11": {
      "order_quantity": 110,
      "distances": [
        2499,
        664,
        1697,
        1566,
        1551,
        814,
        2004,
        665,
        241,
        1753,
        2572,
        0,
        1826,
        1036,
        493,
        1199,
        849,
        1728,
        1754,
        2078
      ]
    },
    "n12": {
      "order_quantity": 110,
      "distances": [
        769,
        2280,
        497,
        1004,
        645,
        1382,
        178,
        2377,
        1953,
        1191,
        1046,
        1826,
        0,
        2232,
        1689,
        1843,
        1721,
        1766,
        1868,
        552
      ]
    },
    "n13": {
      "order_quantity": 90,
      "distances": [
        1463,
        1600,
        2633,
        2502,
        2487,
        1750,
        2054,
        1601,
        1177,
        2689,
        1536,
        1036,
        2232,
        0,
        543,
        2135,
        1785,
        692,
        718,
        1680
      ]
    },
    "n14": {
      "order_quantity": 50,
      "distances": [
        2006,
        1057,
        2090,
        1959,
        1944,
        1207,
        1511,
        1058,
        634,
        2146,
        2079,
        493,
        1689,
        543,
        0,
        1592,
        1242,
        1235,
        1261,
        1585
      ]
    },
    "n15": {
      "order_quantity": 90,
      "distances": [
        2516,
        535,
        1346,
        839,
        1198,
        461,
        2021,
        534,
        958,
        652,
        2589,
        1199,
        1843,
        2135,
        1592,
        0,
        350,
        1745,
        1771,
        2095
      ]
    },
    "n16": {
      "order_quantity": 110,
      "distances": [
        2394,
        559,
        1224,
        717,
        1076,
        339,
        1899,
        656,
        608,
        904,
        2467,
        849,
        1721,
        1785,
        1242,
        350,
        0,
        1623,
        1649,
        1973
      ]
    },
    "n17": {
      "order_quantity": 90,
      "distances": [
        997,
        2182,
        2167,
        2036,
        2021,
        1284,
        1588,
        2279,
        1855,
        2223,
        844,
        1728,
        1766,
        692,
        1235,
        1745,
        1623,
        0,
        102,
        1214
      ]
    },
    "n18": {
      "order_quantity": 70,
      "distances": [
        1099,
        2208,
        2269,
        2138,
        2123,
        1386,
        1690,
        2305,
        1881,
        2325,
        822,
        1754,
        1868,
        718,
        1261,
        1771,
        1649,
        102,
        0,
        1316
      ]
    },
    "n19": {
      "order_quantity": 110,
      "distances": [
        421,
        2532,
        953,
        1256,
        897,
        1634,
        374,
        2629,
        2205,
        1443,
        494,
        2078,
        552,
        1680,
        1585,
        2095,
        1973,
        1214,
        1316,
        0
      ]
    }
  },
  "restaurants": {
    "r0": {
      "neighbourhood_distance": [
        797,
        2156,
        563,
        880,
        521,
        1258,
        302,
        2253,
        1829,
        1067,
        884,
        1702,
        162,
        2070,
        1527,
        1719,
        1597,
        1604,
        1706,
        390
      ],
      "restaurant_distance": [
        0
      ]
    }
  },
  "vehicles": {
    "v0": {
      "start_point": "r0",
      "speed": "INF",
      "capacity": 600
    }
  }
        # ... (your input data)
    }
    target_vehicle = "v0" 
    vehicle_capacity = input_data["vehicles"][target_vehicle]["capacity"]
    num_paths = 3 
    G = create_graph(input_data)
    output = solve_cvrp(G, vehicle_capacity, num_paths, target_vehicle)
    # Correct the paths in the output
    for v, paths in output.items():
        for path, nodes in paths.items():
            # Correct the paths to start and end at the depot ("r0")
            output[v][path] = ["r0"] + nodes + ["r0"]
    with open('level1a_output.json', 'w') as json_file:
        json.dump(output, json_file)
    print(f'Output written to level1a_output.json for {target_vehicle}')


 