from queue import PriorityQueue
import time
import math
from itertools import product
from utils.functions import tuple_operations, manhattan_distance


def time_it(func):
    def wrapper(*args, **kwargs):
        start = time.time()
        result = func(*args, **kwargs)
        end = time.time()
        print(f"{func.__name__} cost time：{end - start} s")
        return result

    return wrapper


class Node:
    def __init__(self, coord_info, parent=None, edge_cost=0.):
        self.coord_info = coord_info
        self.coord = coord_info[0]
        self.parent = parent
        self.direction = coord_info[1]  # "x", "y", "z"

        if self.parent is None:
            self.depth = 1
            self.n_cp = 0
            self.edge_cost = edge_cost
            self.energy = 1 if coord_info[1][1:] == "z" else 0
        else:
            self.edge_cost = self.parent.edge_cost + edge_cost
            energy = 1 if coord_info[1][1:] == "z" else 0
            self.energy = self.parent.energy + energy
            if self.parent.direction != self.direction:
                self.n_cp = self.parent.n_cp + 1
                self.depth = self.parent.depth + 1
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
            if x_small <= x <= x_big:
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

    def __init__(self, space_coords: tuple, obstacle_coords: list, w_path: float, w_bend: float, w_energy: float,
                 min_dis_bend: int, gif=False):
        """
        :param space_coords: the diagonal coords of the valid cuboid in an incremental order.
                            for example: ((0, 0), (100, 100), one must be (0, 0, 0).
        :param start: the starting nozzle, example as (1, 1) grid.
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
        self.open_set = dict(zip(self.phy_vertex, [0] * len(self.phy_vertex)))
        self.close_set = dict(zip(self.phy_vertex, [0] * len(self.phy_vertex)))
        self.pq.queue.clear()

    def init_edge_cost(self, edge_cost=1.):
        if self.dim == 3:
            edges = list(product(self.open_set, ["+x", "-x", "+y", "-y", "+z", "-z"]))
        else:
            edges = list(product(self.open_set, ["+x", "-x", "+y", "-y"]))
        self.edge_cost = dict(zip(edges, [edge_cost] * len(edges)))

    def set_directions(self):
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
        if p_coord in self.open_set and self.open_set[p_coord] == 0:
            return True
        return False

    def is_in_close_set(self, p_coord: tuple):
        if self.close_set[p_coord] == 1:
            return True
        return False

    def is_feasible_bend_point(self, p):
        p_n_cp = p.n_cp
        k = 0  # the point number between two bend points
        while p.parent and p.parent.n_cp >= p_n_cp - 1:
            p = p.parent
            k += 1
        # print(list(abs(x - y) for x, y in zip(p_coord, end)))
        return k >= self.min_dis_bend

    def is_enough_space(self, p_coord, direction, radius, delta):
        shift = [math.ceil(radius + delta)] * self.dim
        if direction in ['+x', '-x']:
            shift[0] = 0
        elif direction in ['+y', '-y']:
            shift[1] = 0
        else:
            shift[2] = 0
        p1 = tuple_operations(p_coord, tuple(shift), '-')
        p2 = tuple_operations(p_coord, tuple(shift), '+')
        for item in list(product(*(range(s, e+1) for s, e in zip(p1, p2)))):
            if item not in self.open_set:
                return False
        return True

    def process_point(self, curr_p, end_info):
        curr_p_coord = curr_p.coord
        if curr_p.n_cp == curr_p.parent.n_cp + 1:
            if self.is_feasible_bend_point(curr_p):  # or (self.cmp(curr_p.coord, end_info[0]) and curr_p.direction == end_info[1]) or curr_p.depth == 2
                pass
            else:
                return None  # district the minimum distance between two bend point

        p_cost = self.total_cost(curr_p, end_info[0])
        if not self.is_enough_space(curr_p.coord, curr_p.direction, self.radius, self.delta):
            p_cost = p_cost + 2.

        if self.is_in_open_set(curr_p_coord):
            self.open_set[curr_p_coord] = p_cost
            self.pq.put((p_cost, curr_p))
        elif p_cost < self.open_set[curr_p_coord]:  # update minimum cost and node
            self.open_set[curr_p_coord] = p_cost
            self.pq.queue = [(priority, value) for priority, value in self.pq.queue if value != curr_p]
            self.pq.put((p_cost, curr_p))
        else:
            pass

    def build_path(self, p):
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
        # for example start_info: ((5, 7, 0), "+x"); end_info: ((0, 99, 77), "+y");

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


if __name__ == '__main__':
    import matplotlib.pyplot as plt
    import matplotlib.animation as anime
    from plotting import cuboid
    space_coords = ((0, 0), (10, 10))  # coords
    # obstacle_coord = [[(3, 3), (6, 6)]]
    obstacle_coord = []
    pipes = (((0, 4), "+x"), ((10, 6), "+x"), 1, 0)
    n_pipe = 1
    model = AStar(space_coords, obstacle_coord, w_path=1., w_bend=2., w_energy=1., min_dis_bend=2, gif=True)
    (bend_path, path), info = model.run(pipes[0], pipes[1], pipes[2], pipes[3])
    print(bend_path)
    print("-----\n")
    print(path)
    fig_gif = plt.figure(figsize=(5, 5))
    axes = fig_gif.gca()
    metadata = dict(title="Movie", artist="sourabh")
    writer = anime.PillowWriter(fps=1, metadata=metadata)
    with writer.saving(fig_gif, "Astar.gif", 100):
        for i in range(len(info)):
            point = info[i][-1]
            open_map = info[i][0]
            cuboid.structure_cuboid(space_coords[0], space_coords[1], ax=axes)
            for k in range(len(obstacle_coord)):
                cuboid.shadow_cuboid(obstacle_coord[k][0], obstacle_coord[k][1], ax=axes)
            axes.scatter(*point, marker="o", c="red", alpha=1., s=50)
            nonzero_coords = [value[0] for value in open_map.items() if value[1] > 0]
            for x_ind, y_ind in nonzero_coords:
                axes.text(x_ind-0.2, y_ind + 0.1, f"{open_map[x_ind, y_ind]:.1f}", size=10, color="black")
            axes.set_xlim([space_coords[0][0], space_coords[1][0]])
            axes.set_ylim([space_coords[0][1], space_coords[1][1]])
            axes.set_xticks(range(space_coords[0][0], space_coords[1][0], 1))
            axes.set_yticks(range(space_coords[0][1], space_coords[1][1], 1))
            axes.grid(True)
            axes.set_xlabel("x", fontsize=30)
            axes.set_ylabel("y", fontsize=30)
            writer.grab_frame()
            axes.cla()

