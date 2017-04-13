#!/usr/bin/env python

import numpy as np


def get_centre_of_grasp(fingertips):
    """
    Calculate the centre of the grasp
    """
    n = len(fingertips)
    sum_x = 0.0
    sum_y = 0.0
    sum_z = 0.0

    for keys, values in fingertips.items():
        x, y, z = values
        sum_x += x
        sum_y += y
        sum_z += z

    return (sum_x/n), (sum_y/n), (sum_z/n)


def find_triangle_area(a, b, c):
    """
    Calculates the area of a triangle given the points at its corners
    """
    ab = a - b
    ac = a - c
    cross_product = np.cross(ab, ac)
    triangle_area = np.linalg.norm(cross_product) / 2

    return triangle_area


def poly_3d_area(finger_tips):
    """
    Polygon area in 3D for Hand E (Five fingers)
    """
    polygon_area = 0.0

    xa, ya, za = finger_tips['rh_fftip']  # 1st finger-tip
    xb, yb, zb = finger_tips['rh_mftip']  # 2nd finger-tip
    xc, yc, zc = finger_tips['rh_rftip']  # 3rd finger-tip

    a = np.array([xa, ya, za])
    b = np.array([xb, yb, zb])
    c = np.array([xc, yc, zc])

    polygon_area += find_triangle_area(a, b, c)

    xb, yb, zb = finger_tips['rh_lftip']  # 4nd finger-tip
    b = np.array([xb, yb, zb])

    polygon_area += find_triangle_area(a, b, c)

    xc, yc, zc = finger_tips['rh_thtip']  # 5nd finger-tip
    c = np.array([xc, yc, zc])

    polygon_area += find_triangle_area(a, b, c)

    return polygon_area
