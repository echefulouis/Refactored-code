import networkx as nx
import numpy as np
from networkx.algorithms import approximation as approx
import matplotlib.pyplot as plt
from pylab import figure
from matplotlib.patches import Circle
from matplotlib.patches import Polygon as MPolygon


class HamiltonianPathFinder:
    def __init__(self, nx_graph, vertex_sequence, graph_points,max_sector_visibility_polygon_area):
        self.R = nx_graph
        self.vertex_sequence = vertex_sequence
        self.graph_points = graph_points
        self.max_sector_visibility_polygon_area = max_sector_visibility_polygon_area
        self.vertex_sequence_cords = self.get_vertex_sequence_coords()

    def get_vertex_sequence_coords(self):
        vertex_sequence_cord = {}
        for i in self.vertex_sequence:
            vertex_sequence_cord[i] = tuple(self.graph_points.tolist()[i])
        return vertex_sequence_cord

    def _create_adj_matrix(self):

        mat_dist = np.zeros((len(self.vertex_sequence), len(self.vertex_sequence)))

        for i in range(len(self.vertex_sequence)):
            for j in range(len(self.vertex_sequence)):
                if i != j:
                    path = nx.shortest_path(self.R, source=self.vertex_sequence[i], target=self.vertex_sequence[j])
                    path_length = 0
                    for k in range(len(path) - 1):
                        path_length = path_length + self.R[path[k]][path[k + 1]]['weight']
                    if not path:
                        pass
                    else:
                        mat_dist[i][j] = path_length

        return mat_dist

    def _get_hamiltonian_path_with_ratio_as_weights(self, adj_matrix_distance):
        vertex_node = list(self.vertex_sequence_cords.keys())

        F = nx.Graph()
        for i in range(len(vertex_node)):
            F.add_node(vertex_node[i], pos=self.vertex_sequence_cords[vertex_node[i]])

        for i in range(len(vertex_node)):
            for j in range(len(vertex_node)):
                if i != j:
                    F.add_edge(vertex_node[i], vertex_node[j], weight=((adj_matrix_distance[i][j] / self.max_sector_visibility_polygon_area[vertex_node[i]]) * 100))

        hamiltonian_path = nx.approximation.traveling_salesman_problem(F, cycle=False)
        hamiltonian_path = approx.greedy_tsp(F, source=hamiltonian_path[0])

        total_distance = 0
        hamiltonian_edge_distance = {}
        for i in range(len(hamiltonian_path) - 1):
            hamiltonian_edge_distance[hamiltonian_path[i]] = F[hamiltonian_path[i]][hamiltonian_path[i + 1]]['weight']
            total_distance += F[hamiltonian_path[i]][hamiltonian_path[i + 1]]['weight']

        return F.copy(), hamiltonian_path, hamiltonian_edge_distance

    def _get_hamiltonian_path_with_length(self,adj_matrix_distance):
        vertex_node = list(self.vertex_sequence_cords.keys())

        F = nx.Graph()
        for i in range(len(vertex_node)):
            F.add_node(vertex_node[i], pos=self.vertex_sequence_cords[vertex_node[i]])

        for i in range(len(vertex_node)):
            for j in range(len(vertex_node)):
                if i != j:
                    F.add_edge(vertex_node[i], vertex_node[j], weight=((adj_matrix_distance[i][j])))

        hamiltonian_path = nx.approximation.traveling_salesman_problem(F, cycle=False)
        hamiltonian_path = nx.algorithms.approximation.tsp.greedy_tsp(F, source=hamiltonian_path[0])

        total_distance = 0
        hamiltonian_edge_distance = {}
        for i in range(len(hamiltonian_path) - 1):
            hamiltonian_edge_distance[hamiltonian_path[i]] = F[hamiltonian_path[i]][hamiltonian_path[i + 1]]['weight']
            total_distance += F[hamiltonian_path[i]][hamiltonian_path[i + 1]]['weight']

        return F.copy(), hamiltonian_path, hamiltonian_edge_distance

    def draw_hamiltonian_cycle(self,h_path):
        Z = nx.Graph()
        vertexsequence_edges = []
        vertexsequence_cords = self.get_vertex_sequence_coords()

        for i in range(len(h_path) - 1):
            vertexsequence_edges.append(tuple([h_path[i], h_path[i + 1]]))

        for i in h_path:
            Z.add_node(i, pos=vertexsequence_cords[i])

        Z.add_edges_from(vertexsequence_edges)
        pos = nx.get_node_attributes(Z, 'pos')
        nx.draw(Z, pos, with_labels=True)
        plt.show()

    def draw_hamiltonian_circles_on_main_graph(self,h_path, max_sec_coords,obs1,obs2,obs3,LineList):
        sub_path_dict = {}
        sub_path_edges = []
        nx_graph = self.R
        nx_coords = self.graph_points
        h_path_coords = self.get_vertex_sequence_coords()
        fig = figure(figsize=(18, 16))
        ax = fig.add_subplot(111, aspect='equal', xlabel="S", ylabel="t")
        ax.set_xlim(-2, 300)
        ax.set_ylim(-2, 200)
        for index, num in enumerate(h_path):
            if index + 1 == len(h_path):
                pass
            else:
                x, y = h_path_coords[num][0], h_path_coords[num][1]
                a, b = h_path_coords[h_path[index + 1]][0], h_path_coords[h_path[index + 1]][1]
                unitA = Circle((x, y), 5, facecolor='none', fill=True, color='blue', edgecolor=(0, 0.8, 0.8),
                               linewidth=2,
                               alpha=0.5)
                unitB = Circle((a, b), 5, facecolor='none', fill=True, color='blue', edgecolor=(0, 0.8, 0.8),
                               linewidth=2,
                               alpha=0.5)

                # ax.add_patch(MPolygon(max_sec_coords[num], closed=True, fill=True, color='r', linewidth=0))
                ax.add_patch(MPolygon(LineList, closed=True, fill=False, color='black', label='line 1', linewidth=3))
                ax.add_patch(MPolygon(obs1, closed=True, fill=True, color='gray', label='line 1', linewidth=2))
                ax.add_patch(MPolygon(obs2, closed=True, fill=True, color='gray', label='line 1', linewidth=2))
                ax.add_patch(MPolygon(obs3, closed=True, fill=True, color='gray', label='line 1', linewidth=2))

                ax.add_patch(unitA)

                nx.draw(nx_graph, nx_coords, with_labels=True)
                # Find the shortest path between the nodes

                path = nx.shortest_path(nx_graph, source=h_path[index], target=h_path[index + 1])
                sub_path_dict[h_path[index]] = path
                sub_path_edges.extend(list(zip(path, path[1:])))

                ax.add_patch(unitB)

        nx.draw(nx_graph, nx_coords, edgelist=sub_path_edges, edge_color='r', width=5.0, with_labels=True)
        # print(sub_path_dict)

        return sub_path_dict
