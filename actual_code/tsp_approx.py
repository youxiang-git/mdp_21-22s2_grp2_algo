import networkx as nx
import matplotlib
import numpy as np

def calc_TSP(node_c):
    g1 = nx.complete_graph(len(node_c))

    node_c_no_h = []
    for x in node_c:
        node_c_no_h.append(x[:2])

    for node, node2 in g1.edges:
        xy = np.subtract(node_c_no_h[node], node_c_no_h[node2])
        xy = np.abs(xy)
        xy = np.sum(xy)
        g1[node][node2]['weight'] = xy
    tsp_path = nx.approximation.traveling_salesman_problem(g1, weight = 'weight', cycle = True, method = nx.algorithms.approximation.traveling_salesman.greedy_tsp)
    return tsp_path


# def main():
#     c_list = [(2, 18, 90), (15, 3, 90), (3, 3, 90), (7, 8, 90), (14, 14, 90), (8, 15, 90)]
#     tsp_p = calc_TSP(c_list)
#     print(tsp_p)

# main()