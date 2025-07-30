#!/usr/bin/env python
"""
Solve traveling salesman problems with bound and branch

Implements a variant of the bound and branch algorithm to solve a cluster-based variant of an assymetric traveling salesman problem. This determines the optimal order of components to create the cutting path. The clusters are introduced because ordered list of points comprising a component path can be navigated either forward or backward, but not both.
"""

import itertools
import random
import matplotlib.pyplot as plt
import networkx as nx
import numpy as np
from icecream import ic


def generate_brothers(num_nodes):
    """
    Create brothers (clusters) to track which nodes represent the same component

    Args:
        num_nodes (int): Number of nodes in the problem. Equal to twice the number of components"

    Returns:
        dictionary: Map of node clusters
    """
    if num_nodes % 2 != 0:
        raise ValueError("Anzahl der Knoten muss gerade sein.kk")
    return {i: i + 1 if i % 2 == 0 else i - 1 for i in range(num_nodes)}


def create_distance_matrix(num_nodes, brothers, nodes):
    """
    Create matrix of distances between nodes

    Args:
        num_nodes (int): Number of nodes in the problem. Equal to twice the number of components
        brothers (dictionary): Map of node clusters
        nodes (list): List of all component objects in the scene

    Returns:
        list: Cost to travel between two nodes
    """
    matrix = []
    for i in range(num_nodes):
        row = []
        for j in range(num_nodes):
            if i == j:
                row.append(0)
            elif brothers.get(i) == j:
                row.append(float("inf"))  # keine Verbindung zu Bruder
            else:
                row.append(np.linalg.norm(nodes[i].last_pt - nodes[j].first_pt))
        matrix.append(row)
    return matrix


def calculate_distance(path, matrix):
    dist = 0
    for i in range(len(path) - 1):
        d = matrix[path[i]][path[i + 1]]
        if d == float("inf"):
            return float("inf")
        dist += d
    last_leg = matrix[path[-1]][path[0]]
    if last_leg == float("inf"):
        return float("inf")
    return dist + last_leg


def bnb_tsp(matrix, nodes):
    n = len(nodes)
    best_cost = float("inf")
    best_path = []

    def branch(path, visited, cost_so_far):
        nonlocal best_cost, best_path

        if len(path) == n:
            full_cost = cost_so_far + matrix[path[-1]][path[0]]
            if full_cost < best_cost:
                best_cost = full_cost
                best_path = path[:]
            return

        for next_node in nodes:
            if next_node in visited:
                continue
            prev = path[-1]
            dist = matrix[prev][next_node]
            if dist == float("inf"):
                continue
            new_cost = cost_so_far + dist
            if new_cost >= best_cost:
                continue  # pruning
            path.append(next_node)
            visited.add(next_node)
            branch(path, visited, new_cost)
            path.pop()
            visited.remove(next_node)

    for start in nodes:
        branch([start], {start}, 0)

    return best_path, best_cost


def tsp_branch_and_bound_with_brothers(matrix, brothers):
    """
    Find solution to a clustered, assymetric traveling salesman problem with bound and branch

    Args:
        matrix (list): Matrix of edge costs between nodes
        brothers (dictionary): Maps which nodes belong to which clusters

    Returns:
        tuple:
            - best_overall_path (list): Optimal path
            - best_overall_cost (float): Cost of the path
            - best_selection: Set of nodes selected from clusters
    """
    used = set()
    pairs = []
    for a, b in brothers.items():
        if a not in used and b not in used:
            pairs.append((a, b))
            used.add(a)
            used.add(b)

    best_overall_path = None
    best_overall_cost = float("inf")
    best_selection = None

    for selection in itertools.product(*pairs):
        selected_nodes = list(selection)
        path, cost = bnb_tsp(matrix, selected_nodes)
        if cost < best_overall_cost:
            best_overall_cost = cost
            best_overall_path = path
            best_selection = set(selected_nodes)

    return best_overall_path, best_overall_cost, best_selection


def plot_graph(matrix, brothers, best_path, selected_nodes):
    G = nx.Graph()
    n = len(matrix)

    color_palette = [
        "tab:blue",
        "tab:orange",
        "tab:green",
        "tab:red",
        "tab:purple",
        "tab:brown",
        "tab:pink",
        "tab:olive",
        "tab:cyan",
    ]
    pair_color = {}
    node_colors = {}
    used_pairs = set()
    color_index = 0

    for a, b in brothers.items():
        pair = tuple(sorted((a, b)))
        if pair not in used_pairs:
            color = color_palette[color_index % len(color_palette)]
            used_pairs.add(pair)
            pair_color[pair] = color
            color_index += 1

        for node in pair:
            node_colors[node] = color if node in selected_nodes else "lightgray"

    for i in range(n):
        G.add_node(i)

    for i in range(n):
        for j in range(i + 1, n):
            if brothers.get(i) == j:
                continue
            if matrix[i][j] != float("inf"):
                G.add_edge(i, j, weight=matrix[i][j])

    edge_colors = []
    green_edges = set()
    for i in range(len(best_path)):
        u = best_path[i]
        v = best_path[(i + 1) % len(best_path)]
        green_edges.add((min(u, v), max(u, v)))

    for u, v in G.edges():
        edge_colors.append(
            "green" if (min(u, v), max(u, v)) in green_edges else "lightgray"
        )

    pos = nx.spring_layout(G, seed=42)
    nx.draw(
        G,
        pos,
        with_labels=True,
        node_color=[node_colors[n] for n in G.nodes()],
        edge_color=edge_colors,
        node_size=700,
        width=2,
    )

    edge_labels = nx.get_edge_attributes(G, "weight")
    nx.draw_networkx_edge_labels(G, pos, edge_labels=edge_labels)
    plt.title("Branch and Bound TSP mit Brüderpaaren")
    plt.show()


# Hauptprogramm
if __name__ == "__main__":
    num_nodes = 14
    random.seed(1)

    brothers = generate_brothers(num_nodes)
    matrix = create_distance_matrix(num_nodes, brothers)

    best_path, best_cost, selected_nodes = tsp_branch_and_bound_with_brothers(
        matrix, brothers
    )
    print("BnB - Bester Pfad:", best_path)
    print("Distanz:", best_cost)
    plot_graph(matrix, brothers, best_path, selected_nodes)
