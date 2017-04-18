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
    n = len(finger_tips)
    if n < 3:
        return 0

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


def find_cosine_angle(a, b, c):
    """ Finds angle at b for points a, b and c """
    ba = a - b
    bc = c - b

    cosine_angle = np.dot(ba, bc) / (np.linalg.norm(ba) * np.linalg.norm(bc))
    angle = np.arccos(cosine_angle)

    return np.degrees(angle)


def measure_grasp_polygon_angles(fingertips):
    """
    Computes the angles of polygon at each finger-tip
    """
    n = len(fingertips)
    if n < 3:
        return 0

    # Make list with fingertip points in order '/rh_fftip', '/rh_mftip', '/rh_rftip', '/rh_lftip', '/rh_thtip',
    tips_list = [fingertips['rh_fftip'], fingertips['rh_mftip'], fingertips['rh_rftip'], fingertips['rh_lftip'],
                 fingertips['rh_thtip']]

    ideal_angle = (180 * (n-2) / n)
    max_angle = ((n-2) * (180 - ideal_angle) * 2*ideal_angle)

    summation = 0.0

    for idx, (x, y, z) in enumerate(tips_list):

        xa, ya, za = tips_list[(idx - 1)]
        xb, yb, zb = tips_list[idx]
        if idx == (len(tips_list) - 1):  # if passing last element in list, set to first
            xc, yc, zc = tips_list[0]
        else:
            xc, yc, zc = tips_list[(idx + 1)]

        a = np.array([xa, ya, za])
        b = np.array([xb, yb, zb])
        c = np.array([xc, yc, zc])

        angle = find_cosine_angle(a, b, c)
        summation += abs(angle - ideal_angle)

    measure = summation/max_angle

    return measure


def main():
    """ TEST for find_triangle_area """
    # TODO(): yugigi
    a = np.array([0.0, 0.0, 1.0])
    b = np.array([2.0, 3.0, 1.0])
    c = np.array([4.0, 0.0, 1.0])
    print find_cosine_angle(a, b, c)


if __name__ == '__main__':
    main()