#!/usr/bin/env python 
# -*- coding: utf-8 -*-
# @Time    : 2023/6/2 9:52
# @Author  : Leijun Ye

from model.Astar import AStar
import unittest
import matplotlib.pyplot as plt
from plotting import cuboid, trace


class test_case(unittest.TestCase):
    def test_dim2_free_space(self):
        space_coords = ((0, 0), (10, 10))  # coords
        start = (0, 2)
        end = (8, 0)  # grip id
        model = AStar(space_coords, start)
        obstacle_coord = [[(2, 4), (3, 8)], [(3, 0), (5, 2)]]
        model.explore_obstacle(obstacle_coord)
        model.set_energy(obstacle_coord)
        path = model.run(end)

        fig = plt.figure(figsize=(10, 10))
        ax = fig.add_subplot()
        ax.set_aspect("auto")
        cuboid.structure_cuboid(space_coords[0], space_coords[1], ax=ax)
        for k in range(len(obstacle_coord)):
            cuboid.shadow_cuboid(obstacle_coord[k][0], obstacle_coord[k][1], ax=ax)
        trace.plot_trace2(path, ax=ax)
        ax.set_xticks(range(space_coords[0][0], space_coords[1][0], 1))
        ax.set_yticks(range(space_coords[0][1], space_coords[1][1], 1))
        ax.grid(True)
        ax.set_xlabel("x", fontsize=30)
        ax.set_ylabel("y", fontsize=30)
        ax.set_xticklabels([])
        ax.set_yticklabels([])
        fig.show()

    def _test_dim2_with_obstacle(self):
        space_coords = ((0, 0, 0), (10, 10, 10))  # coords
        start = (0, 2, 2)
        end = (8, 6, 4)  # grip id
        model = AStar(space_coords, start)
        obstacle_coord = [[(0, 4, 2), (4, 9, 6)], [(3, 0, 2), (5, 2, 6)], [(4, 4, 2), (6, 5, 6)]]
        model.explore_obstacle(obstacle_coord)
        # model.set_energy(obstacle_coord, distance=3)
        path = model.run(end)
        # print(path)

        fig = plt.figure(figsize=(10, 10))
        ax = fig.add_subplot(projection='3d')
        ax.set_aspect("auto")
        cuboid.structure_cuboid(space_coords[0], space_coords[1], ax=ax)
        for k in range(len(obstacle_coord)):
            cuboid.shadow_cuboid(obstacle_coord[k][0], obstacle_coord[k][1], ax=ax)
        trace.plot_trace3(path, ax=ax)
        # ax.view_init(30, 60)
        # ax.axis("off")

        ax.set_xlabel("x")
        ax.set_ylabel("y")
        ax.set_zlabel("z")
        # ax.set_xticks([])
        # ax.set_yticks([])
        # ax.set_zticks([])
        # for angle in range(0, 360):
        #     ax.view_init(30, angle)
        #     plt.draw()
        #     plt.pause(.001)
        fig.show()


if __name__ == '__main__':
    unittest.main()