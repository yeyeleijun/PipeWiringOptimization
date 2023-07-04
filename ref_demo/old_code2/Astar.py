import numpy as np
from queue import PriorityQueue
from ref_demo.old_code2.Point import Node
import time
from itertools import product
import math
from utils.functions import tuple_operations, generate_rectangle_vertices, manhattan_distance


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

    @staticmethod
    def get_max_distance(p1: tuple, p2: tuple):
        res = 0
        for x, y in zip(p1, p2):
            res = max(res, abs(x - y))
        return res

    def __init__(self, space_coords: tuple, w_path: float, w_bend: float, w_energy: float, max_energy: float, min_dis_bend: int, gif=False):
        """
        :param space_coords: the diagonal coords of the valid cuboid in an incremental order.
                            for example: ((0, 0), (100, 100), one must be (0, 0, 0).
        :param start: the starting nozzle, example as (1, 1) grid.
        """
        self.space_coords = space_coords
        self.grid_size = tuple(space_coords[1][i] - space_coords[0][i] for i in range(len(space_coords[0])))
        self.dim = len(space_coords[0])
        self.open_set = np.zeros(self.grid_size, dtype=np.float16)
        self.close_set = np.zeros(self.grid_size, dtype=np.float16)
        self.dir_map = np.zeros(self.grid_size, dtype=np.float16)
        self.pq = PriorityQueue()
        self.free_grid = np.ones(self.grid_size, dtype=np.uint8)  # 1 is valid
        self.set_directions()
        self.energy = np.ones(self.grid_size, dtype=np.float16) * max_energy
        self.w_path = w_path
        self.w_bend = w_bend
        self.w_energy = w_energy
        self.min_dis_bend = min_dis_bend
        self.gif = gif

    def reinit(self):
        self.open_set = np.zeros(self.grid_size, dtype=np.float16)
        self.close_set = np.zeros(self.grid_size, dtype=np.float16)
        self.dir_map = np.zeros(self.grid_size, dtype=np.float16)
        self.pq.queue.clear()

    def set_directions(self):
        if self.dim == 3:
            self.directions = [(0, 1, 0), (0, -1, 0), (1, 0, 0), (-1, 0, 0), (0, 0, 1), (0, 0, -1)]
        else:
            self.directions = [(0, 1), (0, -1), (1, 0), (-1, 0)]

    def explore_obstacle(self, obstacle_coords, tolerance=0):
        """
        :param obstacle_coords: list of all obstacles organized as tuples.
        :param tolerance: the space should be extended outward by a certain distance, default is 0.
        """
        for i in range(len(obstacle_coords)):
            coord0, coord1 = obstacle_coords[i]
            coord0 = tuple(map(lambda item: math.floor(item) - tolerance, coord0))
            coord1 = tuple(map(lambda item: math.ceil(item) + tolerance, coord1))
            for coord in product(*(range(s, e) for s, e in zip(coord0, coord1))):
                if self.coord_valid(coord, self.space_coords[0], self.space_coords[1]):
                    self.free_grid[coord] = 0
        return None

    def set_energy(self, obstacle_coords, values, distance=3):

        # set energy along the obstacle, extending 'distance' steps. [1, 2, 3] -> [5, 3, 1]
        assert len(values) == distance
        values = values[::-1]
        for j, dis in enumerate(range(distance, 0, -1)):
            for ind in range(len(obstacle_coords)):
                for i in range(self.dim):
                    coord0, coord1 = obstacle_coords[ind]
                    coord0 = list(map(lambda item: math.floor(item) - dis, coord0))
                    coord1 = list(map(lambda item: math.ceil(item) - 1 + dis, coord1))
                    vertex_coord = generate_rectangle_vertices(coord0, coord1, dimension=self.dim)
                    for diag_coord in vertex_coord:
                        if self.coord_valid(diag_coord, self.space_coords[0], self.space_coords[1]):
                            self.energy[tuple(diag_coord)] = values[j] * math.sqrt(2)
                    fixed_coord0 = coord0.pop(i)
                    fixed_coord1 = coord1.pop(i)
                    coord0 = [x + 1 for x in coord0]
                    for coord_new in product(*(range(s, e) for s, e in zip(coord0, coord1))):
                        coord_new1 = list(coord_new)
                        coord_new1.insert(i, fixed_coord0)
                        if self.coord_valid(coord_new1, self.space_coords[0], self.space_coords[1]):
                            self.energy[tuple(coord_new1)] = values[j]
                        coord_new2 = list(coord_new)
                        coord_new2.insert(i, fixed_coord1)
                        if self.coord_valid(coord_new2, self.space_coords[0], self.space_coords[1]):
                            self.energy[tuple(coord_new2)] = values[j]
        # if self.dim == 2:
        #     self.energy[0, :] = 1
        #     self.energy[-1, :] = 1
        #     self.energy[:, 0] = 1
        #     self.energy[:, -1] = 1
        # elif self.dim == 3:
        #     # init the surface this 3d cuboid
        #     self.energy[0, :, :] = 1
        #     self.energy[-1, :, :] = 1
        #     self.energy[:, 0, :] = 1
        #     self.energy[:, -1, :] = 1
        #     self.energy[:, :, 0] = 1
        #     self.energy[:, :, -1] = 1
        self.energy[np.where(self.free_grid == 0)] = float('inf')
        return None

    def base_cost(self, p):
        f = self.w_path * p.depth + self.w_bend * p.n_cp + self.w_energy * p.energy
        return f

    def heuristic_cost(self, p_coord, end):
        # Manhattan distance between current point and end point
        return manhattan_distance(p_coord, end)

    def total_cost(self, p):
        # print(p.coord, self.energy[p.coord], self.base_cost(p), self.heuristic_cost(p.coord, end))
        # print(list(map(lambda x: self.heuristic_cost(p.coord, x), self.end_nodes)))
        return self.base_cost(p) + min(list(map(lambda x: self.heuristic_cost(p.coord, x), self.end_nodes)))

    def is_valid_point(self, p_coord: tuple):
        if self.coord_valid(p_coord, self.space_coords[0], self.space_coords[1]) and self.free_grid[p_coord]:
            return True
        else:
            return False

    def is_in_open_set(self, p_coord: tuple):
        if self.open_set[p_coord] > 0:
            return True
        return False

    def is_in_close_set(self, p_coord: tuple):
        if self.close_set[p_coord] == 1:
            return True
        return False

    def is_feasible_bend_point(self, p):
        p_n_cp = p.n_cp
        k = -1  # the point number between two bend points
        while p.parent and p.parent.n_cp >= p_n_cp - 1:
            p = p.parent
            k += 1
        # print(list(abs(x - y) for x, y in zip(p_coord, end)))
        return k >= self.min_dis_bend

    def is_enough_space(self, p):
        p_coord = p.coord
        prev_p_coord = p.parent.coord
        m = 0
        for k in range(self.dim):
            if prev_p_coord[k] != p_coord[k]:
                m = k
                break
        shift = [self.diameter] * self.dim
        shift[m] = 0
        p1 = tuple_operations(p_coord, tuple(shift), '-')
        p2 = tuple_operations(p_coord, tuple(shift), '+')
        for item in list(product(*(range(s, e+1) for s, e in zip(p1, p2)))):
            if not self.is_valid_point(item):
                return False

        return True

    def process_point(self, curr_p):
        curr_p_coord = curr_p.coord
        if curr_p.n_cp == curr_p.parent.n_cp + 1 and not self.is_feasible_bend_point(curr_p):
            return None  # district the minimum distance between two bend point

        if not self.is_enough_space(curr_p):
            return None  # there is enough space for pipes with diameter > grid size

        p_cost = self.total_cost(curr_p)
        if not self.is_in_open_set(curr_p_coord):
            self.open_set[curr_p_coord] = p_cost
            self.pq.put((p_cost, curr_p))
        elif p_cost < self.open_set[curr_p_coord]:  # update minimum cost and node
            self.open_set[curr_p_coord] = p_cost
            self.pq.queue = [(priority, value) for priority, value in self.pq.queue if value != curr_p]
            self.pq.put((p_cost, curr_p))
        else:
            pass

    def get_covering_list(self, path, radius=1, delta_k=0):
        Pk = path[:]
        Lk = path[:]  # Initialize Lk as the same as Pk

        for v0 in Pk:
            for v in Lk:
                for direction in self.directions:
                    v_prime = tuple_operations(v, direction, '+')  # Get adjacent vertices of v
                    if self.is_valid_point(v_prime) and self.get_max_distance(v0, v_prime) <= radius + delta_k and v_prime not in Lk:
                        Lk.append(v_prime)
        return Lk

    def build_path(self, p):
        path = []
        bend_point = [p.coord]
        while True:
            path.insert(0, p.coord)  # Insert first
            if p.parent is None:  # p is start point
                bend_point.insert(0, p.coord)
                break
            if p.n_cp == p.parent.n_cp + 1:
                bend_point.insert(0, p.parent.coord)
            p = p.parent
        return path, bend_point
        
    def run(self, start_nodes, end_nodes, diameter=3):
        start_time = time.time()
        if isinstance(start_nodes, tuple):
            start_nodes = [start_nodes]
        if isinstance(end_nodes, tuple):
            end_nodes = [end_nodes]

        for k in range(len(start_nodes)):
            start_node = start_nodes[k]
            start_p = Node(start_node, energy=self.energy[start_node])
            self.pq.put((0, start_p))

        self.end_nodes = end_nodes
        self.diameter = diameter

        detailed_info = []
        while not self.pq.empty():
            # find the node with minimum cost
            curr_p = self.pq.get()[1]
            # print(f'Process Point: {curr_p.coord}')
            if curr_p.coord in end_nodes:  # exit if finding the end point
                return self.build_path(curr_p), detailed_info
            self.close_set[curr_p.coord] = 1
            self.open_set[curr_p.coord] = 0

            pre_p = curr_p
            for direction in self.directions:
                curr_p_coord = tuple_operations(pre_p.coord, direction, '+')
                if not self.is_valid_point(curr_p_coord):
                    continue  # Do nothing for invalid point
                if self.is_in_close_set(curr_p_coord):
                    continue  # Do nothing for visited point
                curr_p = Node(curr_p_coord, parent=pre_p, energy=self.energy[curr_p_coord])
                self.process_point(curr_p)
            if self.gif:
                detailed_info.append((self.open_set.copy(), pre_p.coord))
        end_time = time.time()
        print(f"Simulation time {end_time - start_time :.3f}")