import numpy as np
from models.intersection import Intersection
from config.settings import ROAD_COORDINATES, ROAD_CONNECTIONS, DEFAULT_Z

class RoadNetwork:
    def __init__(self, default_z=DEFAULT_Z):
        self._nodes = {}
        self._adj = {}
        self.default_z = default_z
        self._create_road_network()

    def add_intersection(self, intersection_id, x, y, z=None):
        z = self.default_z if z is None else z
        self._nodes[intersection_id] = Intersection(intersection_id, x, y, z)
        self._adj.setdefault(intersection_id, [])

    def add_connection(self, intersection_a, intersection_b, bidirectional=True):
        if intersection_a not in self._nodes or intersection_b not in self._nodes:
            return
        if intersection_b not in self._adj[intersection_a]:
            self._adj[intersection_a].append(intersection_b)
        if bidirectional and intersection_a not in self._adj[intersection_b]:
            self._adj[intersection_b].append(intersection_a)

    def neighbors(self, intersection_id):
        return list(self._adj.get(intersection_id, []))

    def random_neighbor(self, intersection_id):
        neighbors = self.neighbors(intersection_id)
        if not neighbors:
            return None
        return np.random.choice(neighbors)

    def coords(self, intersection_id):
        node = self._nodes.get(intersection_id)
        return None if node is None else node.coords

    def _create_road_network(self):
        for intersection_id, (x, y) in ROAD_COORDINATES.items():
            self.add_intersection(intersection_id, x, y)

        for intersection_a, intersection_b in ROAD_CONNECTIONS:
            self.add_connection(intersection_a, intersection_b)