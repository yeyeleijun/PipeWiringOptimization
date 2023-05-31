import matplotlib.pyplot as plt
import numpy as np
from itertools import product, combinations


def structure_cuboid(coord1, coord2, ax=None):
    assert len(coord1) == len(coord2) == 3
    l = abs(coord2[0] - coord1[0])
    w = abs(coord2[1] - coord1[1])
    h = abs(coord2[2] - coord1[2])
    center = [(coord2[0] + coord1[0]) / 2, (coord2[1] + coord1[1]) / 2, (coord2[2] + coord1[2]) / 2]
    half_row, half_column, half_layer = l / 2, w / 2, h / 2
    r1 = tuple([-half_row, half_row])
    r2 = tuple([-half_column, half_column])
    r3 = tuple([-half_layer, half_layer])
    if ax is None:
        fig = plt.figure(figsize=(10, 10))
        ax = fig.gca(projection='3d')
        ax.set_aspect("auto")
    for s, e in combinations(np.array(list(product(r1, r2, r3))), 2):
        s = np.array(center) + np.array(s)
        e = np.array(center) + np.array(e)
        # ax.scatter3D(*center, color="r")
        if np.linalg.norm(s - e) == 2 * r1[1] or np.linalg.norm(s - e) == 2 * r2[1] or np.linalg.norm(s - e) == 2 * r3[
            1]:
            ax.plot3D(*zip(s, e), color="purple", lw=1., alpha=0.7)
    return ax


def shadow_cuboid(coord1, coord2, ax=None, stride=1):
    if ax is None:
        fig = plt.figure(figsize=(10, 10))
        ax = fig.gca(projection='3d')
        ax.set_aspect("auto")
    assert len(coord1) == len(coord2) == 3
    l = abs(coord2[0] - coord1[0])
    w = abs(coord2[1] - coord1[1])
    h = abs(coord2[2] - coord1[2])
    o = coord1 if coord1[2] < coord2[2] else coord2
    x = [[o[0], o[0] + l, o[0] + l, o[0], o[0]],
         [o[0], o[0] + l, o[0] + l, o[0], o[0]],
         [o[0], o[0] + l, o[0] + l, o[0], o[0]],
         [o[0], o[0] + l, o[0] + l, o[0], o[0]]]
    y = [[o[1], o[1], o[1] + w, o[1] + w, o[1]],
         [o[1], o[1], o[1] + w, o[1] + w, o[1]],
         [o[1], o[1], o[1], o[1], o[1]],
         [o[1] + w, o[1] + w, o[1] + w, o[1] + w, o[1] + w]]
    z = [[o[2], o[2], o[2], o[2], o[2]],
         [o[2] + h, o[2] + h, o[2] + h, o[2] + h, o[2] + h],
         [o[2], o[2], o[2] + h, o[2] + h, o[2]],
         [o[2], o[2], o[2] + h, o[2] + h, o[2]]]
    x = np.array(x)
    y = np.array(y)
    z = np.array(z)
    ax.plot_surface(x, y, z, color='b', rstride=stride, cstride=stride, alpha=0.1)
    return ax


if __name__ == '__main__':
    coord_outer = [(0, 0, 0), (200, 80, 50)]
    # diagonal obstacle
    coords = [[(30, 30, 30), (50, 40, 40)],
              [(100, 30, 30), (150, 50, 50)],
              [(50, 60, 0), (150, 80, 20)],
              [(150, 0, 0), (190, 30, 40)]]
    fig = plt.figure(figsize=(10, 10))
    ax = fig.add_subplot(111, projection='3d')
    ax.set_aspect("auto")
    structure_cuboid(coord_outer[0], coord_outer[1], ax=ax)
    for i in range(len(coords)):
        ax = shadow_cuboid(coords[i][0], coords[i][1], ax=ax)
    ax.view_init(30, 60)
    # ax.axis("off")
    ax.set_xlabel("x")
    ax.set_ylabel("y")
    ax.set_zlabel("z")
    ax.set_xticks([])
    ax.set_yticks([])
    ax.set_zticks([])
    fig.show()