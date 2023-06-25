"""
main script to execute the multi-pipe routing optimization algorithm.
"""

import math
import time
from queue import PriorityQueue
from utils.functions import tuple_operations, manhattan_distance


# graph setting of G=(V, E), where V is the virtual node and E is the combination of virtual edges
# and replicas of physical initial edges.

# initialize the 3D grid space, assuring that source and destination points of each service are nodes of
# the grid. Each physical node v in the initial graph is replaced by an exploited node.


class Node:
    def __init__(self, coord_info, parent=None, energy=None):
        self.id = coord_info
        self.coord = coord_info[0]
        self.parent = parent
        if self.id[1] == "x":
            self.neighbor = [(tuple_operations(self.coord, (1, 0, 0), "+"), "x"),
                             (tuple_operations(self.coord, (1, 0, 0), "-"), "x"),
                             (self.coord, "y"), (self.coord, "z")]
        elif self.id[1] == "y":
            self.neighbor = [(tuple_operations(self.coord, (0, 1, 0), "+"), "y"),
                             (tuple_operations(self.coord, (0, 1, 0), "-"), "y"),
                             (self.coord, "z"), (self.coord, "x")]
        else:
            self.neighbor = [(tuple_operations(self.coord, (0, 0, 1), "+"), "z"),
                             (tuple_operations(self.coord, (0, 0, 1), "-"), "z"),
                             (self.coord, "x"), (self.coord, "y")]

        if self.parent is None:
            self.depth = 1
            self.n_cp = 0
            self.energy = 1 if coord_info[1] == "z" else 0
        else:
            energy = 1 if coord_info[1] == "z" else 0
            self.energy = self.parent.energy + energy
            if self.parent.id[1] != self.id[1]:
                self.n_cp = self.parent.n_cp + 1
                self.depth = self.parent.depth
            else:
                self.n_cp = self.parent.n_cp
                self.depth = self.parent.depth + 1

    def __lt__(self, other):
        return self.coord[0] < other.coord[0]

    def __eq__(self, other):
        return self.coord == other.coord


class AStar:

    @staticmethod
    def cmp(p1: tuple, p2: tuple):
        for x, y in zip(p1, p2):
            if x != y:
                return False
        return True

    @staticmethod
    def coord_valid(coord, coord_lb, coord_rt):
        for x, x_small, x_big in zip(coord, coord_lb, coord_rt):
            if x_small <= x < x_big:
                continue
            else:
                return False
        return True

    def __init__(self, space_coords: tuple, obstacle_coords: list, w_path: float, w_bend: float, w_energy: float,
                 min_dis_bend: int, ):
        """
        :param space_coords: the diagonal coords of the valid cuboid in an incremental order.
                            for example: ((0, 0), (100, 100).
        :param start: the starting nozzle, example as (1, 1) grid.
        """
        self.space_coords = space_coords
        self.obstacle_coords = obstacle_coords
        self.physical_vertex(self.space_coords, self.obstacle_coords, compensate=0)
        self.virtual_vertex()
        self.dim = len(space_coords[0])
        self.open_set = dict(zip(self.vir_vertex, [0] * len(self.vir_vertex)))
        self.close_set = dict(zip(self.vir_vertex, [0] * len(self.vir_vertex)))
        self.pq = PriorityQueue()
        self.w_path = w_path
        self.w_bend = w_bend
        self.w_energy = w_energy
        self.min_dis_bend = min_dis_bend

    def physical_vertex(self, coords: tuple, obstacle_coords: list, compensate=0):
        num_obstacles = len(obstacle_coords)
        self.phy_vertex = []
        for i in range(coords[0][0], coords[1][0] + 1):
            for j in range(coords[0][1], coords[1][1] + 1):
                for k in range(coords[0][2], coords[1][2] + 1):
                    is_valid = True
                    for count in range(num_obstacles):
                        coord0, coord1 = obstacle_coords[count]
                        coord_lb = tuple(map(lambda item: math.floor(item) - compensate, coord0))
                        coord_rt = tuple(map(lambda item: math.ceil(item) + compensate, coord1))
                        if self.coord_valid((i, j, k), coord_lb, coord_rt):
                            is_valid = False
                            break
                        else:
                            continue
                    if is_valid:
                        self.phy_vertex.append((i, j, k))  # add vertex (i, j, k)

    def virtual_vertex(self, ):
        # Each vertex is accompanied by 3 virtual vertex.
        self.vir_vertex = []
        for item in self.phy_vertex:
            for direction in ["x", "y", "z"]:
                self.vir_vertex.append((item, direction))

    def base_cost(self, p):
        f = self.w_path * p.depth + self.w_bend * p.n_cp + self.w_energy * p.energy
        return f

    def heuristic_cost(self, p_coord, end):
        # Manhattan distance between current point and end point
        return manhattan_distance(p_coord, end)

    def total_cost(self, p, end):
        # print(p.coord, self.energy[p.coord], self.base_cost(p), self.heuristic_cost(p.coord, end))
        return self.base_cost(p) + self.heuristic_cost(p.coord, end)

    def is_in_open_set(self, p_coord: tuple):
        if self.open_set[p_coord] > 0:
            return True
        return False

    def is_in_close_set(self, p_id: tuple):
        if self.close_set[p_id] == 1:
            return True
        return False

    def is_feasible_bend_point(self, p):
        p_n_coord = p.coord
        p_n_cp = p.n_cp
        while p.parent and p.parent.n_cp >= p_n_cp - 1:
            p = p.parent
        distance = manhattan_distance(p_n_coord, p.coord)
        return distance >= self.min_dis_bend

    def process_point(self, curr_p, end):
        curr_p_coord = curr_p.coord

        # avoid continuous bending and ensure minimum length of two subsequent bends.
        if curr_p.n_cp == curr_p.parent.n_cp + 1 and not self.is_feasible_bend_point(curr_p):
            return None

        p_cost = self.total_cost(curr_p, end)
        if not self.is_in_open_set(curr_p.id):
            self.open_set[curr_p_coord] = p_cost
            self.pq.put((p_cost, curr_p))
        elif p_cost < self.open_set[curr_p_coord]:  # update minimum cost and node
            self.open_set[curr_p_coord] = p_cost
            self.pq.queue = [(priority, value) for priority, value in self.pq.queue if value != curr_p]
            self.pq.put((p_cost, curr_p))
        else:
            pass

    def build_path(self, p):
        path = []
        path.insert(0, p.id)  # Insert first
        while True:
            if self.cmp(p.coord, self.start.coord):
                break
            else:
                if p.n_cp == p.parent.n_cp + 1:
                    path.insert(0, p.parent.id)
                p = p.parent
        path.insert(0, self.start.id)
        return path

    def run(self, start, end):
        # for example start: ((5, 7, 0), "x"); end: ((0, 99, 77), "y");
        start_time = time.time()

        self.start = Node(start, energy=None)
        self.pq.put((0, self.start))

        while not self.pq.empty():
            # find the node with minimum cost
            curr_p = self.pq.get()[1]
            # print(f'Process Point: {curr_p.coord}')
            if self.cmp(curr_p.coord, end[0]) and curr_p.id[1] == end[1]:  # exit if finding the end point
                return self.build_path(curr_p)

            self.close_set[curr_p.id] = 1
            self.open_set[curr_p.id] = 0

            pre_p = curr_p
            for curr_p_id in pre_p.neighbor:
                if curr_p_id[0] not in self.phy_vertex:
                    continue  # Do nothing for invalid point
                if self.is_in_close_set(curr_p_id):
                    continue  # Do nothing for visited point
                curr_p = Node(curr_p_id, parent=pre_p, energy=None)
                self.process_point(curr_p, end[0])
        end_time = time.time()
        print(f"Simulation time {end_time - start_time :.3f}")


if __name__ == '__main__':
    space_coords = ((0, 0, 0), (6, 6, 6))  # coords
    obstacle_coord = [[(1, 1, 0), (2.5, 2.5, 3.2)]]
    start = ((0, 0, 2), "x")
    end = ((5, 5, 4), "z")
    n_pipe = 1
    model = AStar(space_coords, obstacle_coord, w_path=1, w_bend=1, w_energy=1, min_dis_bend=2)
    path = model.run(start, end)
    print(path)
