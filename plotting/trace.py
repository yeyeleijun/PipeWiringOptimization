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


def plot_trace2(path, ax, c):
    if ax is None:
        fig = plt.figure(figsize=(10, 10))
        ax = fig.gca()
        ax.set_aspect("auto")
    for i in range(len(path)):
        rec = Rectangle(path[i], 1, 1, color=c, alpha=0.4)
        ax.add_patch(rec)

    return ax