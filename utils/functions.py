#!/usr/bin/env python 
# -*- coding: utf-8 -*-
# @Time    : 2023/6/7 14:53
# @Author  : Leijun Ye

def tuple_operations(t1, t2, operator):
    """
    This function implements the addition, subtraction, multiplication, and division between Tuples
    and between Tuple and number.

    Args:
    t1: tuple type, the Tuple to be operated, the length needs to be the same as t2.
    t2: tuple or int/float type, the Tuple or a constant to be operated.
    operator: str type, the operator, including +, -, *, /.

    Returns:
    result: tuple type, the result of the operation. When operator is +-,
            the length of the result is the same as that of t1 and t2;
            when operator is */ and t2 is int/float,
            the length of the result is the same as that of t1.
    """
    if isinstance(t2, int) or isinstance(t2, float):
        # When t2 is int or float type, execute number multiplication and division operations
        if operator == '+':
            result = tuple([i + t2 for i in t1])
        elif operator == '-':
            result = tuple([i - t2 for i in t1])
        elif operator == '*':
            result = tuple([i * t2 for i in t1])
        elif operator == '/':
            if t2 == 0:
                raise ValueError("Cannot division by zero!")
            result = tuple([i / t2 for i in t1])
        else:
            raise ValueError("Invalid operator for numeric types!")
    elif isinstance(t2, tuple):
        # When t2 is tuple type, execute Tuple addition and subtraction operations
        if len(t1) != len(t2):
            raise ValueError("The length of two Tuples is not equal!")
        if operator == '+':
            result = tuple(x + y for x, y in zip(t1, t2))
        elif operator == '-':
            result = tuple(x - y for x, y in zip(t1, t2))
        elif operator == '*':
            result = tuple(x * y for x, y in zip(t1, t2))
        elif operator == '/':
            result = tuple(x / y for x, y in zip(t1, t2))
        else:
            raise ValueError("Invalid operator for Tuple types!")
    else:
        raise TypeError("Unsupported type of t2! Only Tuple, int, and float are supported.")
    return result

def generate_rectangle_vertices(point1, point2, dimension):
    if dimension == 2:
        x1, y1 = point1
        x2, y2 = point2

        vertices = [
            [x1, y1],
            [x1, y2],
            [x2, y1],
            [x2, y2]
        ]
    elif dimension == 3:
        x1, y1, z1 = point1
        x2, y2, z2 = point2

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
    else:
        raise ValueError("Invalid dimension. Only 2 or 3 dimensions are supported.")

    return vertices


def manhattan_distance(t1: tuple, t2: tuple):
    # Manhattan distance between two arbitrary points
    distance = (abs(x - y) for x, y in zip(t1, t2))
    return sum(distance)