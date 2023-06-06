import numpy as np
import matplotlib.pyplot as plt

def plot_trace3(path, ax):
    if ax is None:
        fig = plt.figure(figsize=(10, 10))
        ax = fig.gca(projection='3d')
        ax.set_aspect("auto")
    for i in range(len(path) -1):
        s, e = path[i], path[i+1]
        ax.plot3D(*zip(s, e), color="orange", lw=2., alpha=0.6)
    return ax

def plot_trace2(path, ax):
    if ax is None:
        fig = plt.figure(figsize=(10, 10))
        ax = fig.gca()
        ax.set_aspect("auto")
    ax.scatter(*(np.array(path[0]) + 0.5), marker="o", s=25, c="red")
    ax.text(path[0][0] + 0.1, path[0][1] + 0.2, "start", size=15, color="red")
    ax.scatter(*(np.array(path[-1]) + 0.5), marker="s", s=25, c="red")
    ax.text(path[-1][0] + 0.1, path[-1][1] + 0.2, "end", size=15, color="red")
    for i in range(len(path) -1):
        s, e = np.array(path[i])+0.5, np.array(path[i+1])+0.5
        ax.plot(*zip(s, e), color="orange", lw=5., alpha=0.6)
    return ax