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


# def main():
#     """ TEST for find_triangle_area """
#     a = np.array([0.0, 0.0, 1.0])
#     b = np.array([2.0, 3.0, 1.0])
#     c = np.array([3.0, 0.0, 1.0])
#     print find_triangle_area(a, b, c)


def find_triangle_area(a, b, c):
    """
    Calculates the area of a triangle given the points at its corners
    """
    ab = a - b  # np.dot(a, b)
    ac = a - c  # np.dot(a, c)

    cross_product = np.cross(ab, ac)

    triangle_area = np.linalg.norm(cross_product) / 2

    return triangle_area

# ''' ALTERNATIVE: Have not yet got working correctly '''
# def find_triangle_area(a, b, c):
#     ab = abs(a - b)  # np.dot(a, b)
#     ac = abs(a - c)  # np.dot(a, c)
#     print ab, ac
#
#     triangle_area = 0.2 * np.sqrt((ab[1]*ac[2] - ab[2]*ac[1])**2 + (ab[2]*ac[0] - ab[0]*ac[2])**2 +
#                                   (ab[0]*ac[1] - ab[1]*ac[0])**2)
#     return triangle_area


def poly_3d_area(finger_tips):
    """
    Polygon area in 3D for Hand E (Five fingers)
    """
    polygon_area = 0.0

    # 'rh_fftip', 'rh_mftip', 'rh_rftip',    'rh_lftip', 'rh_thtip'

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

""" SUPERSEDED BY poly_3d_area
Computes the polygon area of the grasp
"""
# def convert_points_to_Xs_and_Ys(trans):
#     """
#     Convert points to cartesian coordinates
#
#     (dict) -> return X, Y, Z
#     """
#     Xs = np.array([])
#     Ys = np.array([])
#     #Zs = np.array([])
#
#     for key, (x, y, z) in trans.items():
#         Xs = np.append(Xs, x)
#         Ys = np.append(Ys, y)
#         # Zs = np.append(Zs, z)
#
#     return Xs, Ys  #, Zs
#
# def poly_area((Xs, Ys)):
#     """
#     Implementation of Shoelace formula
#
#     (X, Y, Z) -> return (float)
#     """
#     return 0.5 * np.abs(np.dot(Xs, np.roll(Ys, 1)) - np.dot(Ys, np.roll(Xs, 1)))
#
# def measure_grasp_polygon_area(trans):
#     """
#     Calculates polygon area
#
#     () -> return (float)
#     """
#     return poly_area(convert_points_to_Xs_and_Ys(trans))


def main():
    print 1


def find_cosine_angle(a, b, c):
    """ NOT TESTED """
    ba = a - b
    bc = c - b

    cosine_angle = np.dot(ba, bc) / (np.linalg.norm(ba) * np.linalg.norm(bc))
    angle = np.arccos(cosine_angle)

    return angle  # np.degrees(angle)


def measure_grasp_polygon_angles(finger_tips):
    """
    Computes the angles of polygon at each finger-tip
    """
    n = len(finger_tips)
    print n

    ideal_angle = (180 * (n-2) / n)

    max_angle = ((n-2) * (180 - ideal_angle) * 2*ideal_angle)

    summation = 2

    measure = summation/max_angle

    return measure

if __name__ == '__main__':
    main()
