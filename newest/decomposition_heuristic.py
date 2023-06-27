#!/usr/bin/env python 
# -*- coding: utf-8 -*-
# @Time    : 2023/6/25 15:38
# @Author  : Leijun Ye

from Astar import AStar
from utils.functions import tuple_operations
from itertools import product


class decomposition_heuristic(AStar):
    def __init__(self, maxit, space_coords, obstacle_coords, pipes, w_path, w_bend, w_energy, max_energy, min_dis_bend):
        super(decomposition_heuristic, self).__init__(space_coords, obstacle_coords, w_path, w_bend, w_energy, max_energy, min_dis_bend, gif=False)
        self.maxit = maxit
        self.n_pipes = len(pipes)
        self.index_category = ['I_par'] * int(0.1*maxit) + ['I_cluster'] * int(0.8*maxit) + ['I_seq' * int(0.1*maxit)]
        self.edge_cost = dict()
        self.K0 = []  # list composed of the start coord and end coord of pipes, [[(start_coord1), (end_coord1), diameter1, delta1], ...]

    def get_covering_list(self, path, radius=1, delta=0):
        Pk = []
        for item in path:
            Pk.append(item[0])
        Lk = path[:]  # Initialize Lk as the same as Pk

        for v0 in Pk:
            for v in Lk:
                for direction in self.directions:
                    v_prime = tuple_operations(v, direction[0], '+')  # Get adjacent vertices of v
                    if self.is_in_open_set(v_prime) and self.get_max_distance(v0, v_prime) <= radius + delta and v_prime not in Lk:
                        Lk.append(v_prime)
        Lk = list(product(Lk, [direction[1] for direction in self.directions]))
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
        pass

    def update_cost(self, edges: list):
        pass

    def cmp_priority(self, pipe_i, pipe_j):
        pass

    def main_run(self):
        stop = 0
        it = 0
        Kit = self.K0
        Kbar = self.K0
        covering_list_n = [[] for _ in range(self.n_pipes)]
        while stop == 0 and it <= self.maxit:
            if self.index_category[it] == 'I_seq':
                Kit = self.K0
                Kbar = self.K0

            for pipe_k in Kit:
                k = self.get_pipe_index(pipe_k)
                _, path_k = self.run(pipe_k[0], pipe_k[1], radius=pipe_k[2], delta=pipe_k[3])
                covering_list_n[k] = self.get_covering_list(path_k, radius=pipe_k[2], delta=pipe_k[3])
                if self.index_category[it] == 'I_seq':
                    Kbar.remove(pipe_k)
                    self.update_cost(covering_list_n[k])

            cov_conflict = [[[] for _ in range(self.n_pipes)] for _ in range(len(Kit))]
            for pipe_k in Kit:
                k = self.get_pipe_index(pipe_k)
                for pipe_k_prime in self.K0:
                    k_prime = self.get_pipe_index(pipe_k_prime)
                    cov_conflict[k][k_prime] = list(set(covering_list_n[k]) & set(covering_list_n[k_prime]))

            if not self.is_empty_list(cov_conflict):
                stop = 0
                if self.index_category[it] == 'I_par':
                    for k in range(len(cov_conflict)):
                        cov_conflict_set = set(item for item in cov_conflict[k])
                        self.update_cost(list(cov_conflict_set))
                if self.index_category[it] == 'I_cluster':
                    Kit_next = set()
                    for i, pipe_i in enumerate(Kit):
                        for j, pipe_j in enumerate(self.K0):
                            if cov_conflict[i][j]:
                                if self.cmp_priority(pipe_i, pipe_j):
                                    Kit_next.add(pipe_j)
                                else:
                                    Kit_next.add(pipe_i)

                    temp = set()
                    for pipe_k in Kit_next:
                        k = self.get_pipe_index(pipe_k)
                        for pipe_k_prime in self.K0:
                            k_prime = self.get_pipe_index(pipe_k_prime)
                            if k != k_prime:
                                temp.add(cov_conflict[k][k_prime])
                    self.update_cost(temp)
                    Kit = list(Kit_next)
                it += 1
            else:
                stop = 1
        return covering_list_n


if __name__ == '__main__':
    space_coords = ((0, 0, 0), (12, 12, 12))  # coords
    obstacle_coord = [[(1, 1, 0), (2.5, 2.5, 3.2)]]
    pipes = [(((0, 0, 2), "+x"), ((5, 5, 4), "+z"), 2, 0), (((0, 2, 0), "+y"), ((12, 5, 4), "+y"), 1, 0)]
    maxit = 10
    model = decomposition_heuristic(maxit, space_coords, obstacle_coord, pipes, w_path=1., w_bend=1., w_energy=1., max_energy=5, min_dis_bend=2)
    model.main_run()











