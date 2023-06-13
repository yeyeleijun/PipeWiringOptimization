import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
from utils.functions import get_parallel_line, generate_path_from_bend_points


def plot_trace3(path, ax):
    if ax is None:
        fig = plt.figure(figsize=(10, 10))
        ax = fig.gca(projection='3d')
        ax.set_aspect("auto")
    for i in range(len(path) -1):
        s, e = path[i], path[i+1]
        ax.plot3D(*zip(s, e), color="orange", lw=2., alpha=0.6)
    return ax


def plot_trace2(path, bend_points, ax, c, diameter):
    if ax is None:
        fig = plt.figure(figsize=(10, 10))
        ax = fig.gca()
        ax.set_aspect("auto")
    bend_points_all = [bend_points]
    bend_points_all.append(get_parallel_line(bend_points, 'left_up'))
    bend_points_all.append(get_parallel_line(bend_points, 'right_down'))
    for bend_points in bend_points_all:
        path = generate_path_from_bend_points(bend_points)
        for i in range(len(path)):
            rec = Rectangle(path[i], diameter, diameter, color=c, alpha=0.2)
            ax.add_patch(rec)
        for i in range(len(bend_points) - 1):
            s, e = np.array(bend_points[i])+0.5, np.array(bend_points[i+1])+0.5
            ax.plot(*zip(s, e), color=c, lw=5., alpha=0.6)
    return ax