#!/usr/bin/env python 
# -*- coding: utf-8 -*-
# @Time    : 2023/6/2 9:52
# @Author  : Leijun Ye
import numpy as np
import unittest
import matplotlib.pyplot as plt
from mayavi import mlab
from model.Astar import AStar
from plotting import cuboid, trace, dim3plot
from itertools import product
import matplotlib.animation as anime
from utils.functions import tuple_operations
color_list = ["#DE47AB", "#72BE97", "#F7F797", "#7C749B", "#E85726"]


class test_case(unittest.TestCase):
    def test_dim2_with_obstacle(self):
        space_coords = ((0, 0), (100, 100))  # coords
        obstacle_coord = [[(20, 30), (40, 60)], [(50, 20), (70, 80)]]
        start = [(0, 50), (0, 10)]
        end = [(99, 90), (99, 30)]  # grip id
        n_pipe = 2
        gif = False
        model = AStar(space_coords, w_path=1, w_bend=1, w_energy=1, max_energy=20, min_dis_bend=2, gif=gif)
        energy_value_obstacle = np.arange(3, 7)
        energy_value_path = np.repeat([1, 5], 5)

        # show building and obstacle position
        fig = plt.figure(figsize=(10, 10))
        ax = fig.add_subplot()
        ax.set_aspect("auto")
        cuboid.structure_cuboid(space_coords[0], space_coords[1], ax=ax)
        for k in range(len(obstacle_coord)):
            cuboid.shadow_cuboid(obstacle_coord[k][0], obstacle_coord[k][1], ax=ax)
        for k in range(1):
            ax.text(start[k][0] + 0.1, start[k][1] + 0.2, "start", size=15, color=color_list[k])
            ax.text(end[k][0] + 0.1, end[k][1] + 0.2, "end", size=15, color=color_list[k])
        ax.set_xlim([space_coords[0][0], space_coords[1][0]])
        ax.set_ylim([space_coords[0][1], space_coords[1][1]])
        ax.set_xticks(range(space_coords[0][0], space_coords[1][0], 1))
        ax.set_yticks(range(space_coords[0][1], space_coords[1][1], 1))
        ax.grid(True)
        ax.set_xlabel("x", fontsize=30)
        ax.set_ylabel("y", fontsize=30)
        ax.set_xticklabels([])
        ax.set_yticklabels([])

        # run model
        model.explore_obstacle(obstacle_coord)
        model.set_energy(obstacle_coord, values=energy_value_obstacle, distance=len(energy_value_obstacle))
        paths = []
        bend_points = []
        for pipe_i in range(n_pipe):
            print(f"Processing pipe: {pipe_i}")
            (path, bend_point), info = model.run(start[pipe_i], end[pipe_i])
            if pipe_i < n_pipe - 1:
                model.free_grid[tuple(zip(*path))] = 0
                for factor in range(1, len(energy_value_path), 1):
                    neighboors = [tuple_operations(item, tuple_operations(direction, factor, '*'), '+') for item in path for direction in model.directions]
                    for item in neighboors:
                        if model.is_valid_point(item):
                            model.energy[item] = min(energy_value_path[factor], model.energy[item])
                model.energy[tuple(zip(*path))] = float('inf')
            model.reinit()
            paths.append(path)
            bend_points.append(bend_point)
            print(bend_point)

        # plot gif, showing the searching process
        if gif:
            fig_gif = plt.figure(figsize=(10, 10))
            axes = fig_gif.gca()
            metadata = dict(title="Movie", artist="sourabh")
            writer = anime.PillowWriter(fps=1, metadata=metadata)
            with writer.saving(fig_gif, "Astar.gif", 100):
                for i in range(len(info[-50:])):
                    point = info[-50+i][-1]
                    open_map = info[-50+i][0]
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

        # show the searched path
        for j, path in enumerate(bend_points):
            trace.plot_trace2(path, ax=ax, c=color_list[j])
        # energy = model.energy
        # for xx, yy in product(range(energy.shape[0]), range(energy.shape[1])):
        #     ax.text(xx+0.1, yy+0.3, f"{energy[xx, yy]:.1f}", size=10, color="black")
        fig.savefig("trace.png")
        plt.close(fig)

    def _test_dim3_with_obstacle(self):
        space_coords = ((0, 0, 0), (20, 20, 50))  # coords
        obstacle_coord = [[(4, 4, 10), (10, 10, 20)], [(12, 12, 10), (15, 15, 40)]]
        start = [(0, 0, 5), (5, 10, 0)]
        end = [(19, 19, 40), (90, 50, 40)]  # grip id
        n_pipe = 1
        model = AStar(space_coords, w_path=1, w_bend=1, w_energy=1, max_energy=20, min_dis_bend=2)
        energy_value_obstacle = np.arange(3, 8)
        energy_value_path = np.repeat([1, 5], 5)

        # run model
        model.explore_obstacle(obstacle_coord)
        model.set_energy(obstacle_coord, values=energy_value_obstacle, distance=len(energy_value_obstacle))
        paths = []
        bend_points = []
        for pipe_i in range(n_pipe):
            print(f"Processing pipe: {pipe_i}")
            (path, bend_point), info = model.run(start[pipe_i], end[pipe_i])
            if pipe_i < n_pipe - 1:
                model.free_grid[tuple(zip(*path))] = 0
                for factor in range(1, len(energy_value_path), 1):
                    neighboors = [tuple_operations(item, tuple_operations(direction, factor, '*'), '+') for item in path for direction in model.directions]
                    for item in neighboors:
                        if model.is_valid_point(item):
                            model.energy[item] = min(energy_value_path[factor], model.energy[item])
                model.energy[tuple(zip(*path))] = float('inf')
            model.reinit()
            paths.append(path)
            bend_points.append(bend_point)
            print(bend_point)

        fig = mlab.figure(figure=1, size=(400, 350))
        dim3plot.structure_cuboid((0, 0, 0), (20, 20, 50))
        for k in range(len(obstacle_coord)):
            dim3plot.surface_cuboid(obstacle_coord[k][0], obstacle_coord[k][1], name=f"obstacle{k}")
        for k in range(n_pipe):
            dim3plot.trace_plot(bend_points[k])
        mlab.show()


if __name__ == '__main__':
    unittest.main()