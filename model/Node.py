#!/usr/bin/env python 
# -*- coding: utf-8 -*-
# @Time    : 2023/7/5 9:43
# @Author  : Leijun Ye

class Node:
    """
    This class represents a node in a graph.

    :param coord_info: A tuple that contains coordinate information of the node.
                       It has the format ``(coord, direction)``, where ``coord`` is the coordinate
                       and ``direction`` is the direction of the node ("+x", "-x", "+y", "-y", "+z" or "-z").
    :type coord_info: tuple
    :param parent: The parent node of this node. Default is None for the root node.
    :type parent: Node, optional
    :param edge_cost: The cost associated with the edge from the parent node to this node.
    :type edge_cost: float, optional

    :ivar coord_info: Coordinate information of the node.
    :vartype coord_info: tuple
    :ivar coord: The coordinate of the node.
    :vartype coord: tuple
    :ivar parent: The parent node of this node.
    :vartype parent: Node
    :ivar direction: The direction of the node ("+x", "-x", "+y", "-y", "+z" or "-z").
    :vartype direction: str
    :ivar depth: The depth of the node in the graph, i.e., the length of the path from parent node to current node.
    :vartype depth: int
    :ivar n_cp: The number of directional changes in the path from the root node to this node.
    :vartype n_cp: int
    :ivar energy: The energy associated with the path from the root node to this node.
    :vartype energy: int
    """

    def __init__(self, coord_info, parent=None, edge_cost=0.):
        self.coord_info = coord_info
        self.coord = coord_info[0]
        self.parent = parent
        self.direction = coord_info[1]  # "+x", "-x", "+y", "-y", "+z", "-z"

        if self.parent is None:
            self.depth = 1
            self.n_cp = 0
            self.edge_cost = edge_cost
            self.energy = 1 if coord_info[1][1:] == "z" else 0
        else:
            self.edge_cost = self.parent.edge_cost + edge_cost
            energy = 1 if coord_info[1][1:] == "z" else 0
            self.energy = self.parent.energy + energy
            if self.parent.direction != self.direction:
                self.n_cp = self.parent.n_cp + 1
                self.depth = self.parent.depth + 1
            else:
                self.n_cp = self.parent.n_cp
                self.depth = self.parent.depth + 1

    def __lt__(self, other):
        return self.coord[0] < other.coord[0]

    def __eq__(self, other):
        return self.coord == other.coord
