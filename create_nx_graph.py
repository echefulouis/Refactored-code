import networkx as nx
import matplotlib.pyplot as plt
from sklearn.neighbors import NearestNeighbors
from sklearn.metrics.pairwise import pairwise_distances
from shapely.geometry import LineString, MultiPolygon, Polygon
from matplotlib.patches import Polygon as MPolygon
import numpy as np


class NetworkxNeighbours:

    def __init__(self, networkx_points, new_obstacles, num_neigbours):
        self.networkx_points = networkx_points
        self.new_obstacles = new_obstacles
        self.num_neigbours = num_neigbours
        self.graph = nx.Graph()

    def fit_model(self):
        neigh = NearestNeighbors(n_neighbors=self.num_neigbours)
        # Fit the model
        neigh.fit(self.networkx_points)

        # Calculate pairwise distances between points
        distances = pairwise_distances(self.networkx_points, self.networkx_points, metric="euclidean")
        # Create weighted adjacency matrix from distances
        adj_matrix = distances

        # Predict the nearest neighbors
        neighbors = neigh.kneighbors_graph(self.networkx_points, mode='distance')
        self.graph = nx.from_numpy_array(neighbors.toarray())
        # Remove self loop
        self.graph.remove_edges_from(nx.selfloop_edges(self.graph))

        # Add weights to edges
        for i, j in self.graph.edges():
            self.graph[i][j]['weight'] = adj_matrix[i, j]

    def plot_obstacles(self):
        ax = plt.gca()
        for index, i in enumerate(self.new_obstacles):
            g = [j.x for j in i]
            h = [j.y for j in i]

            polygon_patch = MPolygon(np.array([g, h]).T, closed=True, color="black")
            ax.add_patch(polygon_patch)
        # Dictionary for multipolygon
        polygons = {}

        for index, i in enumerate(self.new_obstacles):
            x = [j.x for j in i]
            y = [j.y for j in i]

            polygons[index] = Polygon(np.array([x, y]).T)

        # loop through the dictionary and create a multipolygon
        mp = MultiPolygon([polygons[i] for i in polygons])

        return mp

    def check_collision(self, point1, point2, mp):
        lines = LineString([point1, point2])
        return lines.intersects(mp)

    def remove_collisions(self):
        mp = self.plot_obstacles()
        line_nodes = self.graph.edges()

        # Check for collision between each line and collision object
        for node in line_nodes:
            point1 = (list(self.networkx_points[node[0]])[0], list(self.networkx_points[node[0]])[1])
            point2 = (list(self.networkx_points[node[1]])[0], list(self.networkx_points[node[1]])[1])
            collision = self.check_collision(point1, point2, mp)

            # Check Colllision with Multi-polygon
            if collision:
                self.graph.remove_edge(node[0], node[1])

    def draw_graph(self):
        nx.draw(self.graph, self.networkx_points, with_labels=True, node_size=20)
        plt.show()

    def get_neighbours(self):
        self.fit_model()
        self.remove_collisions()
        self.draw_graph()
        return self.graph.copy()
