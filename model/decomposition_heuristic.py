#!/usr/bin/env python 
# -*- coding: utf-8 -*-
# @Time    : 2023/6/25 15:38
# @Author  : Leijun Ye

from Astar import AStar
from utils.functions import tuple_operations
from plotting.dim3_plot import *
import numpy as np

colors = [
    (0.0, 0.8, 0.8),
    (0.8, 0.8, 0.0),
    (0.8, 0.0, 0.8),
    (0.0, 0.0, 0.8),
    (0.8, 0.0, 0.0),
    (0.0, 0.8, 0.0),
]


class decomposition_heuristic(AStar):
    def __init__(self, maxit, space_coords, obstacle_coords, pipes, w_path, w_bend, w_energy, min_dis_bend, index_category=None):
        super(decomposition_heuristic, self).__init__(space_coords, obstacle_coords, w_path, w_bend, w_energy, min_dis_bend)
        self.maxit = maxit
        self.n_pipes = len(pipes)
        if index_category is None:
            self.index_category = ['I_par'] * int(0.2*maxit) + ['I_cluster'] * int(0.5*maxit) + ['I_seq'] * (maxit - int(0.2 * maxit) - int(0.5 * maxit))
        else:
            self.index_category = index_category
        self.K0 = pipes[:]  # list composed of the start coord and end coord of pipes, [((start_coord1, direction), (end_coord1, direction), diameter1, delta1), ...]

    def get_covering_list(self, path, radius=1, delta=0):
        Pk = []
        for item in path:
            Pk.append(item[0])
        Lk = Pk[:]  # Initialize Lk as the same as Pk

        for v0 in Pk:
            for v in Lk:
                for direction in self.directions:
                    v_prime = tuple_operations(v, direction[0], '+')  # Get adjacent vertices of v
                    if self.is_in_open_set(v_prime) and self.get_max_distance(v0, v_prime) <= radius + delta and v_prime not in Lk:
                        Lk.append(v_prime)
        Lk = set(product(Lk, [direction[1] for direction in self.directions]))
        return Lk

    def find_conflit_edges(self, path1: list, path2: list):
        cov_list1 = self.get_covering_list(path1)
        cov_list2 = self.get_covering_list(path2)
        return list(set(cov_list1) & set(cov_list2))

    @staticmethod
    def is_empty_list(lst):
        for sublist in lst:
            for element in sublist:
                if element:
                    return False
        return True

    def get_pipe_index(self, pipe):
        return self.K0.index(pipe)

    def update_cost(self, edges: list, change_cost):
        for edge in edges:
            self.edge_cost[edge] += change_cost

    def cmp_priority(self, pipe_i, pipe_j):   # the pipe with larger radius has higher priority
        if pipe_i[2] + pipe_i[3] > pipe_j[2] + pipe_j[3]:
            return True
        elif pipe_i[2] + pipe_i[3] >= pipe_j[2] + pipe_j[3]:
            return self.get_pipe_index(pipe_i) > self.get_pipe_index(pipe_j)
        else:
            return False

    def main_run(self):
        stop = 0
        it = 0
        Kit = self.K0[:]
        Kbar = self.K0[:]
        self.covering_list_n = [[] for _ in range(self.n_pipes)]
        path_n = [[] for _ in range(self.n_pipes)]
        bend_points_n = [[] for _ in range(self.n_pipes)]
        while stop == 0 and it < self.maxit:
            print(f"it: {it}| {self.index_category[it]}")
            if self.index_category[it] == 'I_seq':
                Kit = self.K0[:]
                Kbar = self.K0[:]

            for pipe_k in Kit:
                k = self.get_pipe_index(pipe_k)
                # print(f"processing: {k}")
                (bend_points_k, path_k), _ = self.run(pipe_k[0], pipe_k[1], radius=pipe_k[2], delta=pipe_k[3])
                self.reinit()
                # print(f"path: {path_k}")
                path_n[k] = path_k
                bend_points_n[k] = bend_points_k
                self.covering_list_n[k] = self.get_covering_list(path_k, radius=pipe_k[2], delta=pipe_k[3])
                if self.index_category[it] == 'I_seq':
                    Kbar.remove(pipe_k)
                    for item in Kbar:
                        index_item = self.get_pipe_index(item)
                        edges = set(self.covering_list_n[k]) & set(self.covering_list_n[index_item])
                        self.update_cost(list(edges), change_cost=10)

            if it == 0:
                bend_points_n_init = bend_points_n[:]

            self.cov_conflict = [[[] for _ in range(self.n_pipes)] for _ in range(self.n_pipes)]
            for pipe_k in Kit:
                k = self.get_pipe_index(pipe_k)
                for pipe_k_prime in self.K0:
                    k_prime = self.get_pipe_index(pipe_k_prime)
                    if k != k_prime:
                        self.cov_conflict[k][k_prime] = list(set(self.covering_list_n[k]) & set(self.covering_list_n[k_prime]))
            self.cov_conflict_num = np.zeros((self.n_pipes, self.n_pipes))
            for i in range(self.n_pipes):
                for j in range(self.n_pipes):
                    self.cov_conflict_num[i, j] = len(self.cov_conflict[i][j])
            print(f"self.cov_conflict_num: {self.cov_conflict_num / 6}")

            if not self.is_empty_list(self.cov_conflict):
                stop = 0
                if self.index_category[it] == 'I_par':
                    self.cov_conflict_set = set()
                    for k in range(len(self.cov_conflict)):
                        for items in self.cov_conflict[k]:
                            for item in items:
                                self.cov_conflict_set.add(item)
                        self.update_cost(list(self.cov_conflict_set), change_cost=30)
                if self.index_category[it] == 'I_cluster':
                    Kit_next = set()
                    for pipe_i in Kit:
                        i = self.get_pipe_index(pipe_i)
                        for pipe_j in self.K0:
                            j = self.get_pipe_index(pipe_j)
                            if self.cov_conflict[i][j]:
                                if self.cmp_priority(pipe_i, pipe_j):
                                    Kit_next.add(pipe_j)
                                else:
                                    Kit_next.add(pipe_i)

                    conflict_edges = set()
                    for pipe_k in Kit_next:
                        k = self.get_pipe_index(pipe_k)
                        for pipe_k_prime in self.K0:
                            k_prime = self.get_pipe_index(pipe_k_prime)
                            if pipe_k_prime not in Kit_next and self.cov_conflict[k][k_prime]:
                                for item in self.cov_conflict[k][k_prime]:
                                    conflict_edges.add(item)
                    self.update_cost(list(conflict_edges), change_cost=100)
                    Kit = list(Kit_next)
                it = it + 1
            else:
                stop = 1
        return path_n, bend_points_n_init, bend_points_n


if __name__ == '__main__':
    space_coords = ((0, 0, 0), (30, 30, 30))  # coords
    obstacle_coord = []
    # obstacle_coord = [[(10, 10, 10), (20, 20, 20)]]
    # pipes = [(((2, 2, 3), "+y"), ((33, 12, 12), "+x"), 2, 0), (((0, 2, 3), "+x"), ((12, 33, 8), "+z"), 1, 0)]
    pipes = []
    radia = [2, 2, 2, 1, 1]
    for m, coord in enumerate(np.arange(10, 30, 5)):
        pipes.append((((0, coord, 0), "+y"), ((30-coord, 30-coord, 30), "+x"), radia[m], 0))
    maxit = 10
    model = decomposition_heuristic(maxit, space_coords, obstacle_coord, pipes, w_path=1., w_bend=3., w_energy=1.,
                                    min_dis_bend=2)
    paths, bend_points_init, bend_points = model.main_run()

    """plot the initial path"""
    # print(bend_points_init)
    fig = mlab.figure(figure=1, size=(400, 400))
    structure_cuboid(*space_coords, name="space")
    for k in range(len(obstacle_coord)):
        surface_cuboid(*obstacle_coord[k], name=f"obstacle{k}")
    # surface_cuboid((20, 40, 50), (80, 60, 70), name="obstacle2")
    for k in range(len(pipes)):
        trace_plot(bend_points_init[k], color=colors[k], radius_ratio=pipes[k][2], name=f"trace{k}")
    mlab.show()

    """plot the path after solving conflict"""
    print("Done!")
    print(bend_points)
    fig = mlab.figure(figure=1, size=(400, 400))
    structure_cuboid(*space_coords, name="space")
    for k in range(len(obstacle_coord)):
        surface_cuboid(*obstacle_coord[k], name=f"obstacle{k}")
    # surface_cuboid((20, 40, 50), (80, 60, 70), name="obstacle2")
    for k in range(len(pipes)):
        trace_plot(bend_points[k], color=colors[k], radius_ratio=pipes[k][2], name=f"trace{k}")
    mlab.show()





