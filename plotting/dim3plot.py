import os
# os.environ ['ETS_TOOLKIT'] = 'qt4'
import numpy as np
from mayavi import mlab


def surface_cuboid(coord1: tuple, coord2: tuple, name="obstacle1"):
    l = abs(coord2[0] - coord1[0])
    w = abs(coord2[1] - coord1[1])
    h = abs(coord2[2] - coord1[2])
    o = coord1 if coord1[2] < coord2[2] else coord2
    x = [[o[0], o[0] + l, o[0] + l, o[0]],
         [o[0], o[0] + l, o[0] + l, o[0]],
         [o[0], o[0] + l, o[0] + l, o[0]],
         [o[0], o[0] + l, o[0] + l, o[0]]]
    y = [[o[1], o[1], o[1] + w, o[1] + w],
         [o[1], o[1], o[1] + w, o[1] + w],
         [o[1], o[1], o[1], o[1]],
         [o[1] + w, o[1] + w, o[1] + w, o[1] + w]]
    z = [[o[2], o[2], o[2], o[2]],
         [o[2] + h, o[2] + h, o[2] + h, o[2] + h],
         [o[2], o[2], o[2] + h, o[2] + h],
         [o[2], o[2], o[2] + h, o[2] + h]]
    x = np.array(x)
    y = np.array(y)
    z = np.array(z)
    mlab.mesh(x, y, z, name=name, opacity=0.5, representation="surface")


def structure_cuboid(coord1, coord2):
    x1, y1, z1 = coord1
    x2, y2, z2 = coord2
    vertices = [
        [x1, y1, z1],
        [x1, y1, z2],
        [x1, y2, z1],
        [x1, y2, z2],
        [x2, y1, z1],
        [x2, y1, z2],
        [x2, y2, z1],
        [x2, y2, z2]
    ]
    faces = [
        [0, 1, 3, 2],
        [0, 1, 5, 4],
        [0, 2, 6, 4],
        [1, 3, 7, 5],
        [2, 3, 7, 6],
        [4, 5, 7, 6]
    ]
    for face in faces:
        x = [vertices[face[0]][0], vertices[face[1]][0], vertices[face[2]][0], vertices[face[3]][0],
             vertices[face[0]][0]]
        y = [vertices[face[0]][1], vertices[face[1]][1], vertices[face[2]][1], vertices[face[3]][1],
             vertices[face[0]][1]]
        z = [vertices[face[0]][2], vertices[face[1]][2], vertices[face[2]][2], vertices[face[3]][2],
             vertices[face[0]][2]]
        mlab.plot3d(x, y, z, tube_radius=None, opacity=0.7)
    mlab.axes(extent=[x1, x2, y1, y2, z1, z2])


def trace_plot(path):
    xx = [item[0] for item in path]
    yy = [item[1] for item in path]
    zz = [item[2] for item in path]
    mlab.plot3d(xx, yy, zz, tube_radius=1., color=(1., 1., 1.), opacity=0.8)


if __name__ == '__main__':
    fig = mlab.figure(figure=1, size=(400, 350))
    structure_cuboid((0, 0, 0), (100, 100, 100))
    surface_cuboid((0, 0, 0), (10, 10, 40), name="obstacle1")
    surface_cuboid((20, 40, 50), (80, 60, 70), name="obstacle2")
    trace_plot(((0, 0, 40), (50, 0, 40), (50, 100, 40), (100, 100, 40)))
    mlab.show()
