#!/usr/bin/env python 
# -*- coding: utf-8 -*-
# @Time    : 2023/6/2 9:52
# @Author  : Leijun Ye
import numpy as np

from model.Astar import AStar
import unittest
import matplotlib.pyplot as plt
from plotting import cuboid, trace
from itertools import product

import matplotlib.animation as anime


class test_case(unittest.TestCase):
    def test_dim2_free_space(self):
        space_coords = ((0, 0), (10, 10))  # coords
        start = (0, 2)
        end = (8, 0)  # grip id
        model = AStar(space_coords, start)
        obstacle_coord = [[(2, 4), (4, 8)], [(3, 0), (5, 2)]]
        model.explore_obstacle(obstacle_coord)
        model.set_energy(obstacle_coord, 2)
        path, info = model.run(end)

        gif = True
        if gif:
            fig_gif = plt.figure(figsize=(10, 10))
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
                    rect = plt.Rectangle(point, 1, 1, color="yellow", alpha=0.8)
                    axes.add_patch(rect)
                    xx, yy = np.nonzero(open_map)
                    for x_ind, y_ind in zip(xx, yy):
                        axes.text(x_ind+0.1, y_ind+0.3, f"{open_map[x_ind, y_ind]:.1f}", size=10, color="black")
                    axes.set_xlim([space_coords[0][0], space_coords[1][0]])
                    axes.set_ylim([space_coords[0][1], space_coords[1][1]])
                    axes.set_xticks(range(space_coords[0][0], space_coords[1][0], 1))
                    axes.set_yticks(range(space_coords[0][1], space_coords[1][1], 1))
                    axes.grid(True)
                    axes.set_xlabel("x", fontsize=30)
                    axes.set_ylabel("y", fontsize=30)
                    writer.grab_frame()
                    axes.cla()

        fig = plt.figure(figsize=(10, 10))
        ax = fig.add_subplot()
        ax.set_aspect("auto")
        cuboid.structure_cuboid(space_coords[0], space_coords[1], ax=ax)
        for k in range(len(obstacle_coord)):
            cuboid.shadow_cuboid(obstacle_coord[k][0], obstacle_coord[k][1], ax=ax)
        trace.plot_trace2(path, ax=ax)
        energy = model.energy
        for xx, yy in product(range(energy.shape[0]), range(energy.shape[1])):
            ax.text(xx+0.1, yy+0.3, f"{energy[xx, yy]:.1f}", size=10, color="black")
        ax.set_xlim([space_coords[0][0], space_coords[1][0]])
        ax.set_ylim([space_coords[0][1], space_coords[1][1]])
        ax.set_xticks(range(space_coords[0][0], space_coords[1][0], 1))
        ax.set_yticks(range(space_coords[0][1], space_coords[1][1], 1))
        ax.grid(True)
        ax.set_xlabel("x", fontsize=30)
        ax.set_ylabel("y", fontsize=30)
        # ax.set_xticklabels([])
        # ax.set_yticklabels([])
        fig.savefig("trace.png")
        plt.close(fig)



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