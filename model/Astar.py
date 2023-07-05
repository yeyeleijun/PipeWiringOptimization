import math
import time
from itertools import product
from queue import PriorityQueue

from model.Node import Node
from utils.functions import tuple_operations, manhattan_distance


def time_it(func):
    """Decorator that measures the execution time of a function."""

    def wrapper(*args, **kwargs):
        start = time.time()
        result = func(*args, **kwargs)
        end = time.time()
        print(f"{func.__name__} cost time：{end - start} s")
        return result

    return wrapper


class AStar:

    @staticmethod
    def cmp(p1: tuple, p2: tuple):
        """ return True if the tuples are equal element-wise, False otherwise."""
        for x, y in zip(p1, p2):
            if x != y:
                return False
        return True

    @staticmethod
    def coord_valid(coord, coord_lb, coord_rt):
        """Check if the coordinates are valid within the given lower bound and upper bound."""
        for x, x_small, x_big in zip(coord, coord_lb, coord_rt):
            if x_small <= x <= x_big:
                continue
            else:
                return False
        return True

    @staticmethod
    def get_max_distance(p1: tuple, p2: tuple):
        """Get the maximum distance between two tuples element-wise."""
        res = 0
        for x, y in zip(p1, p2):
            res = max(res, abs(x - y))
        return res

    def __init__(self, space_coords: tuple, obstacle_coords: list, w_path: float, w_bend: float, w_energy: float,
                 min_dis_bend: int, gif=False):
        """
        A* pathfinding algorithm implementation. This class is general to 2-dimensional and 3-dimensional spcace.

        :param space_coords: The diagonal coordinates of the valid cuboid in an incremental order.
                            for example: ((0, 0), (100, 100), one must be (0, 0, 0).
        :type space_coords: tuple
        :param obstacle_coords: The list of obstacle coordinates.
        :type obstacle_coords: list
        :param w_path: The weight for the path cost.
        :type w_path: float
        :param w_bend: The weight for the number of bend points (elbows).
        :type w_bend: float
        :param w_energy: The weight for the energy cost.
        :type w_energy: float
        :param min_dis_bend: The minimum required distance between consecutive elbows in the paths.
        :type min_dis_bend: int
        :param gif: Whether to generate a gif (default is False) which can help debug and visualization.
        :type gif: bool
        """

        self.space_coords = space_coords
        self.dim = len(space_coords[0])
        self.obstacle_coords = obstacle_coords
        self.init_property(compensate=0)
        # self.grid_size = tuple(space_coords[1][i] - space_coords[0][i] for i in range(len(space_coords[0])))
        self.init_edge_cost(edge_cost=1.)
        self.pq = PriorityQueue()
        self.set_directions()
        self.w_path = w_path
        self.w_bend = w_bend
        self.w_energy = w_energy
        self.min_dis_bend = min_dis_bend
        self.gif = gif

    def init_property(self, compensate=0):
        num_obstacles = len(self.obstacle_coords)
        phy_vertex = []
        if self.dim == 3:
            for i in range(self.space_coords[0][0], self.space_coords[1][0] + 1):
                for j in range(self.space_coords[0][1], self.space_coords[1][1] + 1):
                    for k in range(self.space_coords[0][2], self.space_coords[1][2] + 1):
                        is_valid = True
                        for count in range(num_obstacles):
                            coord0, coord1 = self.obstacle_coords[count]
                            coord_lb = tuple(map(lambda item: math.floor(item) - compensate, coord0))
                            coord_rt = tuple(map(lambda item: math.ceil(item) + compensate, coord1))
                            if self.coord_valid((i, j, k), coord_lb, coord_rt):
                                is_valid = False
                                break
                            else:
                                continue
                        if is_valid:
                            phy_vertex.append((i, j, k))  # add vertex (i, j, k)
        else:
            for i in range(self.space_coords[0][0], self.space_coords[1][0] + 1):
                for j in range(self.space_coords[0][1], self.space_coords[1][1] + 1):
                    is_valid = True
                    for count in range(num_obstacles):
                        coord0, coord1 = self.obstacle_coords[count]
                        coord_lb = tuple(map(lambda item: math.floor(item) - compensate, coord0))
                        coord_rt = tuple(map(lambda item: math.ceil(item) + compensate, coord1))
                        if self.coord_valid((i, j), coord_lb, coord_rt):
                            is_valid = False
                            break
                        else:
                            continue
                    if is_valid:
                        phy_vertex.append((i, j))
        self.phy_vertex = phy_vertex
        self.open_set = dict(zip(phy_vertex, [0] * len(phy_vertex)))
        self.close_set = dict(zip(phy_vertex, [0] * len(phy_vertex)))

    def reinit(self):
        """Reinitialize the model, used in A* for different pipes"""
        self.open_set = dict(zip(self.phy_vertex, [0] * len(self.phy_vertex)))
        self.close_set = dict(zip(self.phy_vertex, [0] * len(self.phy_vertex)))
        self.pq.queue.clear()

    def init_edge_cost(self, edge_cost=1.):
        """Initialize the cost of edges, which will be changed in decomposition heuristic algorithm."""
        if self.dim == 3:
            edges = list(product(self.open_set, ["+x", "-x", "+y", "-y", "+z", "-z"]))
        else:
            edges = list(product(self.open_set, ["+x", "-x", "+y", "-y"]))
        self.edge_cost = dict(zip(edges, [edge_cost] * len(edges)))

    def set_directions(self):
        """Set the directions based on the dimension."""
        if self.dim == 3:
            self.directions = [((0, 1, 0), "+y"), ((0, -1, 0), "-y"),
                               ((1, 0, 0), "+x"), ((-1, 0, 0), "-x"),
                               ((0, 0, 1), "+z"), ((0, 0, -1), "-z")]
        else:
            self.directions = [((0, 1), "+y"), ((0, -1), "-y"), ((1, 0), "+x"), ((-1, 0), "-x")]

    def base_cost(self, p):
        f = self.w_path * p.edge_cost + self.w_bend * p.n_cp + self.w_energy * p.energy
        return f

    def heuristic_cost(self, p_coord, end):
        # Manhattan distance between current point and end point
        return manhattan_distance(p_coord, end)

    def total_cost(self, p, end):
        return self.base_cost(p) + self.heuristic_cost(p.coord, end)

    def is_in_open_set(self, p_coord: tuple):
        """Check if the given coordinate exists in the open set."""
        if p_coord in self.open_set and self.open_set[p_coord] == 0:
            return True
        return False

    def is_in_close_set(self, p_coord: tuple):
        """Check if the given coordinate exists in the close set."""
        if self.close_set[p_coord] == 1:
            return True
        return False

    def is_feasible_bend_point(self, p):
        """Check whether the given coordinate is a feasible bend point (satisfy the minimum required distance between consecutive elbows) or not."""
        p_n_cp = p.n_cp
        k = 0  # the point number between two bend points
        while p.parent and p.parent.n_cp >= p_n_cp - 1:
            p = p.parent
            k += 1
        # print(list(abs(x - y) for x, y in zip(p_coord, end)))
        return k >= self.min_dis_bend

    def is_enough_space(self, p_coord, direction, radius, delta):
        """
        Check if there is enough space to place the pipe with specified radius and required extra space.

        :param p_coord: The starting coordinate to check.
        :type p_coord: tuple
        :param direction: The direction of the pipe. Possible values are '+x', '-x', '+y', '-y', '+z', '-z'.
        :type direction: str
        :param radius: The radius of the pipe.
        :type radius: float
        :param delta: The required extra space of the pipe.
        :type delta: float
        :return: True if there is enough space in the specified direction, False otherwise.
        :rtype: bool
        """
        shift = [math.ceil(radius + delta)] * self.dim
        if direction in ['+x', '-x']:
            shift[0] = 0
        elif direction in ['+y', '-y']:
            shift[1] = 0
        else:
            shift[2] = 0
        p1 = tuple_operations(p_coord, tuple(shift), '-')
        p2 = tuple_operations(p_coord, tuple(shift), '+')
        for item in list(product(*(range(s, e + 1) for s, e in zip(p1, p2)))):
            if item not in self.open_set:
                return False
        return True

    def process_point(self, curr_p, end_info):
        curr_p_coord = curr_p.coord
        if curr_p.n_cp == curr_p.parent.n_cp + 1:
            if self.is_feasible_bend_point(
                    curr_p):  # or (self.cmp(curr_p.coord, end_info[0]) and curr_p.direction == end_info[1]) or curr_p.depth == 2
                pass
            else:
                return None  # district the minimum distance between two bend point

        p_cost = self.total_cost(curr_p, end_info[0])
        if not self.is_enough_space(curr_p.coord, curr_p.direction, self.radius, self.delta):
            p_cost = p_cost + 2.

        if self.is_in_open_set(curr_p_coord):
            # If the node is not iterated by A*, add it to the priority queue and open set.
            self.open_set[curr_p_coord] = p_cost
            self.pq.put((p_cost, curr_p))
        elif p_cost < self.open_set[curr_p_coord]:
            # If the node is already iterated by A* and has a lower cost, update the minimum cost and node.
            self.open_set[curr_p_coord] = p_cost
            self.pq.queue = [(priority, value) for priority, value in self.pq.queue if value != curr_p]
            self.pq.put((p_cost, curr_p))
        else:
            # Do nothing if the node is already iterated by A* and the cost is larger than the previous edges.
            pass

    def build_path(self, p):
        """Once the end point is found by A*, the founded path and the bend points coordinate along the path is returned."""
        bend_path = []
        path = []
        bend_path.insert(0, p.coord_info)  # Insert first
        while True:
            if self.cmp(p.coord, self.start.coord):
                break
            else:
                path.insert(0, p.coord_info)
                if p.n_cp == p.parent.n_cp + 1:
                    bend_path.insert(0, p.parent.coord_info)
                p = p.parent
        bend_path.insert(0, self.start.coord_info)
        if self.cmp(bend_path[0][0], bend_path[1][0]):
            bend_path.pop(0)
        path.insert(0, self.start.coord_info)
        return bend_path, path

    @time_it
    def run(self, start_info, end_info, radius, delta):
        """
        Run the A* algorithm to find the path of the pipe with given start point and end point.
        :param start_info: for example: ((5, 7, 0), "+x").
        :param end_info: for example: ((0, 99, 77), "+y").
        :param radius: the radius of the pipe.
        :param delta: The required extra space of the pipe.
        :return:
        """
        self.radius = radius
        self.delta = delta
        self.start = Node(start_info, edge_cost=0.)
        self.pq.put((0, self.start))

        detailed_info = []
        while not self.pq.empty():
            # find the node with minimum cost
            curr_p = self.pq.get()[1]
            # print(f'Process Point: {curr_p.coord}')
            if self.cmp(curr_p.coord, end_info[0]):  # exit if finding the end point
                return self.build_path(curr_p), detailed_info
            self.close_set[curr_p.coord] = 1
            self.open_set[curr_p.coord] = 0

            pre_p = curr_p
            for direction in self.directions:
                curr_p_coord = tuple_operations(pre_p.coord, direction[0], '+')
                if curr_p_coord not in self.open_set:
                    continue  # Do nothing for invalid point
                if self.is_in_close_set(curr_p_coord):
                    continue  # Do nothing for visited point
                edge_cost = self.edge_cost[(curr_p_coord, direction[1])]
                curr_p = Node((curr_p_coord, direction[1]), parent=pre_p, edge_cost=edge_cost)
                self.process_point(curr_p, end_info=end_info)
            if self.gif:
                detailed_info.append((self.open_set.copy(), pre_p.coord))
        return (None, None), detailed_info
