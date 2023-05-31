import numpy as np
from queue import PriorityQueue

class Node:
    def __init__(self, data):
        self.data = data
        self.next = None


class LinkedList:
    def __init__(self):
        self.head = None

    def append(self, data):
        new_node = Node(data)
        if self.head is None:
            self.head = new_node
        else:
            current = self.head
            while current.next:
                current = current.next
            current.next = new_node

    def display(self):
        current = self.head
        while current:
            print(current.data, end=" -> ")
            current = current.next
        print("None")


# the grid size of cuboid
M, N, L = 100, 100, 100  # 网格规模

# define start and end cell
start = (99, 0, 99)
end = (0, 46, 0)

# define the direction of pipe
directions = [(0, 1, 0), (0, -1, 0), (1, 0, 0), (-1, 0, 0), (0, 0, 1), (0, 0, -1)]

# define the amplification factor of heuristic distance h(v).
w = 1.0

# The minimum length of two neighbor bends
L_min = 1

dir_map = LinkedList()

# define obstacle-grid and free-grid Sp
Sp = None

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

    pq.put((0, start))
    open_map[start] = 1
    g_s = 0
    bends = [start]


    while not pq.empty():
        _, v = pq.get()
        dir_map.append(v)

        if v == end:
            return construct_path(dir_map, start, end)

        close_map[v] = 1

        for direction in directions:
            neighbor = tuple(v[i] + direction[i] for i in range(3))

            if is_valid(neighbor) and Sp[neighbor] == 0 and close_map[neighbor] == 0:
                v0 = v
                v = neighbor

                if forms_bend(v0, v):
                    d = calculate_distance(v, bends[-1])

                    if d >= L_min:
                        dist = g(v)
                        if open_map[v] == 0:
                            g_v = dist
                            h_v = heuristic(v)
                            f_v = g_v + w * h_v
                            open_map[v] = g_v
                            dir_map.append(v)
                            pq.put((f_v, v))
                        elif open_map[v] > dist:
                            g_v = dist
                            h_v = heuristic(v)
                            f_v = g_v + w * h_v
                            open_map[v] = g_v
                            dir_map.append(v)
                            update_priority_queue(pq, v, f_v)
                else:
                    dist = g(v)
                    if open_map[v] == 0:
                        g_v = dist
                        h_v = heuristic(v)
                        f_v = g_v + w * h_v
                        open_map[v] = g_v
                        dir_map.append(v)
                        pq.put((f_v, v))
                    elif open_map[v] > dist:
                        g_v = dist
                        h_v = heuristic(v)
                        f_v = g_v + w * h_v
                        open_map[v] = g_v
                        dir_map.append(v)
                        update_priority_queue(pq, v, f_v)
                v = v0

    return None

# 判断节点是否在立方体内部
def is_valid(node):
    x, y, z = node
    return 0 <= x < M and 0 <= y < N and 0 <= z < L

# 判断是否形成弯曲
def forms_bend(v0, v):
    # 实现弯曲判断逻辑
    pass

# 计算两个节点之间的距离
def calculate_distance(v0, v):
    # 实现距离计算逻辑
    pass


# 计算节点的实际代价
def g(v):
    # 实现实际代价计算逻辑
    pass

# 更新优先队列中的节点优先级
def update_priority_queue(pq, v, f_v):
    # 实现优先队列更新逻辑
    pq.queue = [(priority, value) for priority, value in pq.queue if value != v]
    pq.put((f_v, v))

# 构造最优路径
def construct_path(dir_map, start, end):
    path = []
    current_node = end
    while current_node != start:
        path.append(current_node)
        current_node = Sp[dir_map[current_node]].id
    path.append(start)
    path.reverse()
    return path

# 运行A*算法并输出结果
path = A_star()
if path:
    print("最优路径：")
    for node in path:
        print(node)
else:
    print("无法找到路径")
