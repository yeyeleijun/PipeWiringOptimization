import unittest

import matplotlib.animation as anime
import matplotlib.pyplot as plt

from model.Astar import AStar
from model.decomposition_heuristic import DecompositionHeuristic
from plotting import cuboid
from plotting.dim3_plot import *

colors = [
    (0.0, 0.8, 0.8),
    (0.8, 0.8, 0.0),
    (0.8, 0.0, 0.8),
    (0.0, 0.0, 0.8),
    (0.8, 0.0, 0.0),
    (0.0, 0.8, 0.0),
]


class MyTestCase(unittest.TestCase):
    def _test_astar(self):
        space_coords = ((0, 0), (10, 10))  # coords
        # obstacle_coord = [[(3, 3), (6, 6)]]
        obstacle_coord = []
        pipes = (((1, 0), "+y"), ((5, 10), "+x"), 1, 0)
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
                    axes.text(x_ind - 0.2, y_ind + 0.1, f"{open_map[x_ind, y_ind]:.1f}", size=10, color="black")
                axes.set_xlim([space_coords[0][0], space_coords[1][0]])
                axes.set_ylim([space_coords[0][1], space_coords[1][1]])
                axes.set_xticks(range(space_coords[0][0], space_coords[1][0], 1))
                axes.set_yticks(range(space_coords[0][1], space_coords[1][1], 1))
                axes.grid(True)
                axes.set_xlabel("x", fontsize=30)
                axes.set_ylabel("y", fontsize=30)
                writer.grab_frame()
                axes.cla()

    def test_decompostion(self):
        space_coords = ((0, 0, 0), (20, 20, 20))  # coords
        obstacle_coord = []
        # obstacle_coord = [[(10, 10, 10), (20, 20, 20)]]
        # pipes = [(((2, 2, 3), "+y"), ((33, 12, 12), "+x"), 2, 0), (((0, 2, 3), "+x"), ((12, 33, 8), "+z"), 1, 0)]
        pipes = []
        radia = [2, 2, 2, 1, 1]
        for m, coord in enumerate(np.arange(2, 20, 4)):
            pipes.append((((0, coord, 0), "+y"), ((20 - coord, 20 - coord, 20), "+x"), radia[m], 0))
        maxit = 10
        index_category = ['I_par'] * int(0.2 * maxit) + ['I_cluster'] * int(0.8 * maxit)
        model = DecompositionHeuristic(maxit, space_coords, obstacle_coord, pipes, w_path=1., w_bend=3., w_energy=0.,
                                       min_dis_bend=0, index_category=index_category)
        paths, bend_points_init, bend_points = model.main_run()

        """plot the initial path"""
        # print(bend_points_init)
        fig = mlab.figure(figure=1, size=(1000, 1000))
        structure_cuboid(*space_coords, name="space")
        for k in range(len(obstacle_coord)):
            surface_cuboid(*obstacle_coord[k], name=f"obstacle{k}")
        # surface_cuboid((20, 40, 50), (80, 60, 70), name="obstacle2")
        for k in range(len(pipes)):
            trace_plot(bend_points_init[k], color=colors[k], radius_ratio=pipes[k][2], name=f"trace{k}")
        # mlab.savefig("./init_pipes.png", magnification=2.0)
        mlab.show()

        """plot the path after solving conflict"""
        print("Done!")
        print(bend_points)
        fig = mlab.figure(figure=1, size=(1000, 1000))
        structure_cuboid(*space_coords, name="space")
        for k in range(len(obstacle_coord)):
            surface_cuboid(*obstacle_coord[k], name=f"obstacle{k}")
        # surface_cuboid((20, 40, 50), (80, 60, 70), name="obstacle2")
        for k in range(len(pipes)):
            trace_plot(bend_points[k], color=colors[k], radius_ratio=pipes[k][2], name=f"trace{k}")
        # mlab.savefig("./optimized_pipes.png", magnification=2.0)
        mlab.show()


if __name__ == '__main__':
    unittest.main()
