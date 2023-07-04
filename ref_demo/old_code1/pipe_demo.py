from queue import PriorityQueue
from itertools import product

# Determine whether in a valid space or not
def is_valid(node):
    x, y, z = node
    return 0 <= x < M and 0 <= y < N and 0 <= z < L


# Determine whether or not is a bend
def forms_bend(v_pre, v_post):
    if v_pre==-1:
        return False
    else:
        distance = tuple(abs(x - y) for x, y in zip(v_pre, v_post))
        if distance.count(1) == 2:
            return True
    return False


# calculate
def calculate_distance(last_bend, v):
    return manhattan_distance(last_bend, v) + 1


# Compute the loss
def g(v, dir_map, start, n_bend, a=1, b=1, c=10):
    f = a * len(construct_path(dir_map, v, start)) + b * n_bend + c * energy[v]   # energy
    return f


# update the priority queue if need to replace element
def update_priority_queue(pq, v, f_v):
    pq.queue = [(priority, value) for priority, value in pq.queue if value != v]
    pq.put((f_v, v))


def manhattan_distance(grid_s, grid_e):
    distance = tuple(abs(x - y) for x, y in zip(grid_s, grid_e))
    return sum(distance)


# 构造最优路径
def construct_path(dir_map, curr_node, start):
    path = []
    while curr_node != start:
        path.append(curr_node)
        curr_node = dir_map[curr_node]
    path.append(start)
    path.reverse()
    return path


def set_energy(Sp, distance=1):
    energy = {(layer, column, row): 25 for layer in range(M) for column in range(N) for row in range(L)}
    directions = [(x, y, z) for x in range(-distance, distance+1) for y in range(-distance, distance+1) for z in range(-distance, distance+1)]
    obstacle_coord = {k: v for k, v in Sp.items() if v == 1}
    for coord in obstacle_coord.keys():
        for direction in directions:
            coord_new = tuple(coord[i] + direction[i] for i in range(3))
            if is_valid(coord_new) and Sp[coord_new] == 1:
                energy[coord_new] = float('inf')
            else:
                energy[coord_new] = 5
    return energy

def explore_sp(M, N, L, obstacle_coords=None):
    Sp = {(layer, column, row): 0 for layer in range(M) for column in range(N) for row in range(L)}
    for x, y in list(product(range(M), range(N))):
        Sp[(x, y, -1)] = 1
        Sp[(x, y, L)] = 1
    for y, z in list(product(range(N), range(L))):
        Sp[(-1, y, z)] = 1
        Sp[(M, y, z)] = 1
    for x, z in list(product(range(M), range(L))):
        Sp[(x, -1, z)] = 1
        Sp[(x, N, z)] = 1
    if obstacle_coords is not None:
        for i in range(len(obstacle_coords)):
            coord0, coord1 = obstacle_coords[i][0], obstacle_coords[i][1]
            for x, y, z in Sp.keys():
                if coord0[0] - 1 <= x <= coord1[0] + 1 and coord0[1] - 1 <= y <= coord1[1] + 1 and coord0[2] - 1 <= z <= \
                        coord1[2] + 1:
                    Sp[(x, y, z)] = 1
    return Sp


# the grid size of cuboid
M, N, L = 10, 10, 10

# define start and end cell
start = (0, 0, 0)
end = (4, 6, 0)
M, N, L = 10, 10, 10  # 网格规模

# define start and end cell
start = (0, 0, 0)
end = (4, 6, 2)

# define the direction of pipe
directions = [(0, 1, 0), (0, -1, 0), (1, 0, 0), (-1, 0, 0), (0, 0, 1), (0, 0, -1)]

# define the amplification factor of heuristic distance h(v).
w = 1.0

# The minimum length of two neighbor bends
L_min = 1

# define obstacle-grid and free-grid Sp
Sp = explore_sp(M, N, L, obstacle_coords=None)  # [(0, 6, 0), (4, 5, 1)]

# define energy
# energy = set_energy(Sp)

# define the heuristic function
def heuristic(node):
    distance = tuple(abs(x - y) for x, y in zip(node, end))
    out = sum(distance)
    return out


# main function of A start algorithm
def A_star():
    pq = PriorityQueue()
    open_map = {(layer, column, row): 0 for layer in range(M) for column in range(N) for row in range(L)}
    close_map = {(layer, column, row): 0 for layer in range(M) for column in range(N) for row in range(L)}
    dir_map = {(layer, column, row): -1 for layer in range(M) for column in range(N) for row in range(L)}
    print("Init done!")
    pq.put((0, start))
    open_map[start] = g(start, dir_map, start, n_bend=0)
    bends = [start]

    while not pq.empty():
        _, v = pq.get()

        if v == end:
            return construct_path(dir_map, end, start), n_bend

        # print(f"exploring {v}")
        close_map[v] = 1
        open_map[v] = 0
        n_bend = 0

        for direction in directions:
            neighbor = tuple(v[i] + direction[i] for i in range(3))

            if is_valid(neighbor) and Sp[neighbor] == 0 and close_map[neighbor] == 0:
                v0 = v
                v = neighbor

                if forms_bend(dir_map[v0], v):
                    d = calculate_distance(v0, bends[-1])
                    if d >= L_min:
                        bends.append(v)
                        n_bend += 1
                        dir_map[v] = v0
                        dist = g(v, dir_map, start, n_bend)
                        if open_map[v] == 0:
                            g_v = dist
                            h_v = heuristic(v)
                            f_v = g_v + w * h_v
                            open_map[v] = g_v
                            pq.put((f_v, v))
                        elif open_map[v] > dist:
                            g_v = dist
                            h_v = heuristic(v)
                            f_v = g_v + w * h_v
                            open_map[v] = g_v
                            update_priority_queue(pq, v, f_v)
                else:
                    dir_map[v] = v0
                    dist = g(v, dir_map, start, n_bend)
                    if open_map[v] == 0:
                        g_v = dist
                        h_v = heuristic(v)
                        f_v = g_v + w * h_v
                        open_map[v] = g_v
                        pq.put((f_v, v))
                    elif open_map[v] > dist:
                        g_v = dist
                        h_v = heuristic(v)
                        f_v = g_v + w * h_v
                        open_map[v] = g_v
                        update_priority_queue(pq, v, f_v)
                print(v, fv)
                v = v0

    return None


# run A*
path, n_bend = A_star()
if path:
    print("n bend: ", n_bend)
    print("optimal path：")
    for node in path:
        print(node)
else:
    print("cant not find valid path")
