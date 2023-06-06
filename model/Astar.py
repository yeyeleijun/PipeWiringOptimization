import numpy as np
from queue import PriorityQueue
import sys
from model.Point import Node
import time
from itertools import product


class AStar:
    @staticmethod
    def manhattan_distance(p1: tuple, p2: tuple):
        # Manhattan distance between two arbitrary points
        distance = (abs(x - y) for x, y in zip(p1, p2))
        return sum(distance)

    @staticmethod
    def cmp(p1: tuple, p2: tuple):
        for x, y in zip(p1, p2):
            if x != y:
                return False
        return True

    @staticmethod
    def add_tuple(p1: tuple, p2: tuple):
        return tuple(x + y for x, y in zip(p1, p2))

    @staticmethod
    def coord_valid(coord, coord_lb, coord_rt):
        for x, x_small, x_big in zip(coord, coord_lb, coord_rt):
            if x_small <= x < x_big:
                continue
            else:
                return False
        return True

    def __init__(self, space_coords: tuple, start: tuple):
        """
        :param space_coords: the diagonal coords of the valid cuboid in an incremental order.
                            for example: ((0, 0), (100, 100), one must be (0, 0, 0).
        :param start: the starting nozzle, example as (1, 1) grid.
        """
        self.space_coords = space_coords
        self.grid_size = tuple(space_coords[1][i] - space_coords[0][i] for i in range(len(space_coords[0])))
        self.dim = len(space_coords[0])
        assert self.dim == len(start)
        self.start = Node(start)
        self.open_set = np.zeros(self.grid_size, dtype=np.float32)
        self.close_set = np.zeros(self.grid_size, dtype=np.float32)
        self.dir_map = np.zeros(self.grid_size, dtype=np.float32)
        self.pq = PriorityQueue()
        self.pq.put((0, self.start))
        self.free_grid = np.ones(self.grid_size, dtype=np.uint8)  # 1 is valid

    def get_directions(self):
        directions = []
        for k in range(self.dim):
            direction = [0] * self.dim
            direction[k] = 1
            directions.append(tuple(direction))
            direction = [0] * self.dim
            direction[k] = -1
            directions.append(tuple(direction))
        return directions

    def explore_obstacle(self, obstacle_coords, tolerance=0):
        """
        :param obstacle_coords: list of all obstacles organized as tuples.
        :param tolerance: the space should be extended outward by a certain distance, default is 0.
        """
        self.obstacle_coords = obstacle_coords
        for i in range(len(obstacle_coords)):
            coord0, coord1 = obstacle_coords[i]
            coord0 = tuple(map(lambda item: int(item) - tolerance, coord0))
            coord1 = tuple(map(lambda item: int(item) + 1 + tolerance, coord1))
            for coord in product(*(range(s, e) for s, e in zip(coord0, coord1))):
                if self.coord_valid(coord, self.space_coords[0], self.space_coords[1]):
                    self.free_grid[coord] = 0

        return None

    def set_energy(self, lb=1, ub=25, step=2):
        thre = (ub - lb) // step
        temp = tuple(map(lambda x: x - thre * 2, self.grid_size))
        energy = np.ones(temp, dtype=np.float32) * ub
        for k in range(lb, step, ub+step):
            assert k > 0
            energy = np.pad(energy, pad_width=1, mode='constant', constant_values=k)

        # update energy
        for i in range(len(self.obstacle_coords)):
            coord0, coord1 = self.obstacle_coords[i]
            obstacle_size = tuple(coord0[i] - coord1[i] for i in range(len(coord0[0])))
            obstacle = np.ones(obstacle_size, dtype=np.float32) * float('inf')
            obstacle = np.pad(obstacle, pad_width=1, mode='constant', constant_values=5)

        return energy

    def base_cost(self, p, w_path=1, w_bend=1, w_energy=1):
        # TODO: add a energy function.
        f = w_path * p.depth + w_bend * p.n_cp + w_energy * self.energy[p.coord]
        return f

    def heuristic_cost(self, p_coord, end):
        # Manhattan distance between current point and end point
        return self.manhattan_distance(p_coord, end)

    def total_cost(self, p, end):
        return self.base_cost(p) + self.heuristic_cost(p.coord, end)

    def is_valid_point(self, p_coord: tuple):
        if self.coord_valid(p_coord, self.space_coords[0], self.space_coords[1]) and self.free_grid[p_coord]:
            return True
        else:
            return False

    def is_in_open_set(self, p: tuple):
        if self.open_set[p] > 0:
            return True
        return False

    def is_in_close_set(self, p: tuple):
        if self.close_set[p] == 1:
            return True
        return False

    def process_point(self, curr_p, end):
        curr_p_coord = curr_p.coord
        if not self.is_valid_point(curr_p_coord):
            return None  # Do nothing for invalid point
        if self.is_in_close_set(curr_p_coord):
            return None  # Do nothing for visited point

        p_cost = self.total_cost(curr_p, end)
        if not self.is_in_open_set(curr_p_coord):
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
        while True:
            path.insert(0, p.coord)  # Insert first
            if self.cmp(p.coord, self.start.coord):
                break
            else:
                p = p.parent
        return path

    def run(self, end):
        start_time = time.time()

        # Process all neighbors
        directions = self.get_directions()
        self.energy = self.set_energy()

        while not self.pq.empty():
            # find the node with minimum cost
            curr_p = self.pq.get()[1]
            print(f'Process Point: {curr_p.coord}')
            if self.cmp(curr_p.coord, end):  # exit if finding the end point
                return self.build_path(curr_p)

            self.close_set[curr_p.coord] = 1
            self.open_set[curr_p.coord] = 0

            pre_p = curr_p
            for direction in directions:
                curr_p_coord = self.add_tuple(pre_p.coord, direction)
                curr_p = Node(curr_p_coord, parent=pre_p)
                self.process_point(curr_p, end)
        end_time = time.time()
        print(f"Simulation time {end_time - start_time :.3f}")
