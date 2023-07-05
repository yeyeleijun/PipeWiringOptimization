#!/usr/bin/env python 
# -*- coding: utf-8 -*-
# @Time    : 2023/6/25 15:38
# @Author  : Leijun Ye

from itertools import product

import numpy as np

from model.Astar import AStar
from utils.functions import tuple_operations


class DecompositionHeuristic(AStar):
    def __init__(self, maxit, space_coords, obstacle_coords, pipes, w_path, w_bend, w_energy, min_dis_bend,
                 index_category=None):
        """
        Initialize a decomposition_heuristic object.

        :param maxit: Maximum number of iterations.
        :type maxit: int
        :param space_coords: Coordinates of the space.
        :type space_coords: tuple
        :param obstacle_coords: Coordinates of the obstacles.
        :type obstacle_coords: list
        :param pipes: Pipes information.
        :type pipes: list
        :param w_path: Weight for path.
        :type w_path: float
        :param w_bend: Weight for bends.
        :type w_bend: float
        :param w_energy: Weight for energy.
        :type w_energy: float
        :param min_dis_bend: Minimum distance for two bend points.
        :type min_dis_bend: int
        :param index_category: Index categories.
        :type index_category: list or None

        `This algorithm was inspired by a previous study. <https://doi.org/10.1016/j.omega.2022.102659>`_.
        """

        super(DecompositionHeuristic, self).__init__(space_coords, obstacle_coords, w_path, w_bend, w_energy,
                                                     min_dis_bend)
        self.maxit = maxit
        self.n_pipes = len(pipes)
        if index_category is None:
            self.index_category = ['I_par'] * int(0.2 * maxit) + ['I_cluster'] * int(0.5 * maxit) + ['I_seq'] * (
                    maxit - int(0.2 * maxit) - int(0.5 * maxit))
        else:
            self.index_category = index_category
        self.K0 = pipes[
                  :]  # list composed of the start coord and end coord of pipes, [((start_coord1, direction), (end_coord1, direction), diameter1, delta1), ...]

    def get_covering_list(self, path, radius=1, delta=0):
        """
        Generate the covering list of the given path.
        Algorithm 4 of the reference.
        """

        Pk = []
        for item in path:
            Pk.append(item[0])
        Lk = Pk[:]  # Initialize Lk as the same as Pk

        for v0 in Pk:
            for v in Lk:
                for direction in self.directions:
                    v_prime = tuple_operations(v, direction[0], '+')  # Get adjacent vertices of v
                    if self.is_in_open_set(v_prime) and self.get_max_distance(v0,
                                                                              v_prime) <= radius + delta and v_prime not in Lk:
                        Lk.append(v_prime)
        Lk = set(product(Lk, [direction[1] for direction in self.directions]))
        return Lk

    def find_conflict_edges(self, path1: list, path2: list):
        """Find the conflict edges between the covering list of two pipes."""
        cov_list1 = self.get_covering_list(path1)
        cov_list2 = self.get_covering_list(path2)
        return list(set(cov_list1) & set(cov_list2))

    @staticmethod
    def is_empty_list(lst):
        """Check if a nested list is empty."""
        for sublist in lst:
            for element in sublist:
                if element:
                    return False
        return True

    def get_pipe_index(self, pipe):
        """Get the index of pipe in K0."""
        return self.K0.index(pipe)

    def update_cost(self, edges: list, change_cost):
        """Update the cost of given edges."""
        for edge in edges:
            self.edge_cost[edge] += change_cost

    def cmp_priority(self, pipe_i, pipe_j):
        """
        Compare the priority of two pipes.

        The priority is determined based on the sum of the radii of the two pipes.
        If the sum of the radii of pipe_i is greater than the sum of the radii of pipe_j,
        then pipe_i has a higher priority.

        In case the sum of the radii is equal for both pipes, the priority is further
        determined by the index of the pipe using the `get_pipe_index` method.
        """

        if pipe_i[2] + pipe_i[3] > pipe_j[2] + pipe_j[3]:
            return True
        elif pipe_i[2] + pipe_i[3] == pipe_j[2] + pipe_j[3]:
            return self.get_pipe_index(pipe_i) > self.get_pipe_index(pipe_j)
        else:
            return False

    def get_bend_distance(self, p1, p2):
        distance = tuple_operations(p2[0], p1[0], '-')
        if p2[1] in ['+x', '-x']:
            m = 0
        elif p2[1] in ['+y', '-y']:
            m = 1
        else:
            m = 2
        return abs(distance[m])

    def pipe_elbow_test(self, pipe_k):
        """
         for each pipe, a shortest path from the source node to the destination node is initially built on the undirected graph
         using A* algorithm. In case the elbow test is passed, returns that path. Otherwise, the part of the path that
         passes the test is kept, the cost of the first elbow that does not pass the test is increased and A* algorithm is run again
         (with the new costs) from the last elbow passing the test to the destination node. The process is repeated every time we find
         an elbow that does not pass the test until the elbow test is passed or until a maximum number of iterations (of the elbow test)
         is reached and then, the elbow test has not been passed.
         Algorithm 3 of the reference.
         ==========
         This function need more test!!!
         ===========
        """

        s, e, radius, delta = pipe_k[0], pipe_k[1], pipe_k[2], pipe_k[3]
        Pk = []
        Bk = []
        test = 1
        it = 0
        maxit = 10
        while test == 1 and it < maxit:
            print(f"it: {it}, Bk: {Bk}, s:{s}, e:{e}")
            (bend_points_k, path_k), _ = self.run(s, e, radius=radius, delta=delta)
            Bk.extend(bend_points_k)
            Pk.extend(path_k)
            for i in range(1, len(bend_points_k)):
                ditance = self.get_bend_distance(bend_points_k[i - 1], bend_points_k[i])
                if ditance < self.min_dis_bend:
                    break
            if i == len(bend_points_k) - 1 and ditance >= self.min_dis_bend:
                test = 0
                return Bk, Pk
            else:
                idx_end = path_k.index(bend_points_k[i - 1])
                Pk = Pk[:idx_end - len(path_k)]
                idx_end = bend_points_k.index(bend_points_k[i - 1])
                Bk = Bk[:idx_end - len(bend_points_k)]
                if bend_points_k[i] in ['+x', '-x']:
                    edges = list(product([bend_points_k[i][0]], ['+x', '-x']))
                elif bend_points_k[i] in ['+y', '-y']:
                    edges = list(product([bend_points_k[i][0]], ['+y', '-y']))
                else:
                    edges = list(product([bend_points_k[i][0]], ['+z', '-z']))
                self.update_cost(edges, change_cost=10)
                s = bend_points_k[i - 1]
            it += 1

    def main_run(self):
        """
        The implementation of decomposition heuristic algorthm which is used for the wiring of multiple pipes simultaneously.
        Algorithm 2 of the reference.
        """

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
                # bend_points_k, path_k = self.pipe_elbow_test(pipe_k)
                (bend_points_k, path_k), _ = self.run(pipe_k[0], pipe_k[1], radius=pipe_k[2], delta=pipe_k[3])
                self.reinit()
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
                        self.cov_conflict[k][k_prime] = list(
                            set(self.covering_list_n[k]) & set(self.covering_list_n[k_prime]))
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
