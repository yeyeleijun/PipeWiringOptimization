#!/usr/bin/env python 
# -*- coding: utf-8 -*-
# @Time    : 2023/6/2 9:52
# @Author  : Leijun Ye

from model.Astar import AStar
import unittest


class test_case(unittest.TestCase):
    def test_dim2_free_space(self):
        space_coords = ((0, 0), (10, 10))  # coords
        start = (0, 2)
        end = (9, 5)  # grip id
        model = AStar(space_coords, start)
        path = model.run(end)
        print(path)


if __name__ == '__main__':
    unittest.main()