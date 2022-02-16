import networkx as nx
import matplotlib
import numpy as np

def calc_TSP(node_c):
    g1 = nx.complete_graph(6)
    for node, node2 in g1.edges:
        # print(node_c[node], node_c[node2])
        # print(node, node2)
        # print(g1[node][node2])
        xy = np.subtract(node_c[node][:2], node_c[node2][:2])
        xy = np.abs(xy)
        # print(xy)
        xy = np.sum(xy)
        g1[node][node2]['weight'] = xy
        # print(g1[node][node2]['weight'])

    # print(g1[0])
    node_c_no_h = []
    for x in node_c:
        node_c_no_h.append(x[:2])

    # nx.draw(g1, node_c_no_h, with_labels = True)
    # labels = nx.get_edge_attributes(g1,'weight')
    # nx.draw_networkx_edge_labels(g1,node_c_no_h,edge_labels=labels)
    tsp_path = nx.approximation.traveling_salesman_problem(g1, weight = 'weight', cycle = True, method = nx.algorithms.approximation.traveling_salesman.greedy_tsp)
    return tsp_path

# def main():
#     c_list = [(2, 16), (5, 2), (3, 9), (11, 7), (16, 4), (16, 13)]
#     tsp_p = calc_TSP(c_list)
#     print(tsp_p)

# main()