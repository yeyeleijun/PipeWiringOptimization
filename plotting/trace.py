import numpy as np
import matplotlib.pyplot as plt

def plot_trace3(path, ax):
    if ax is None:
        fig = plt.figure(figsize=(10, 10))
        ax = fig.gca(projection='3d')
        ax.set_aspect("auto")
    for i in range(len(path) -1):
        s, e = path[i], path[i+1]
        ax.plot3D(*zip(s, e), color="red", lw=2., alpha=0.8)
    return ax

def plot_trace2(path, ax):
    if ax is None:
        fig = plt.figure(figsize=(10, 10))
        ax = fig.gca()
        ax.set_aspect("auto")
    for i in range(len(path) -1):
        s, e = path[i], path[i+1]
        ax.plot(*zip(s, e), color="red", lw=2., alpha=0.8)
    return ax