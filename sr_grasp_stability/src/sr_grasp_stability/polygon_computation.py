#!/usr/bin/env python

import numpy as np

# class Polygon:

"""
Calculate the centre of the grasp
"""
def get_centre_of_grasp(finger_tips):

    n = len(finger_tips)
    sum_x = 0.0
    sum_y = 0.0
    sum_z = 0.0

    for keys, values in finger_tips.items():
        x, y, z = values
        sum_x += x
        sum_y += y
        sum_z += z

    return (sum_x/n), (sum_y/n), (sum_z/n)


"""
Computes the polygon area of the grasp
"""

def convert_points_to_Xs_and_Ys(trans):
    """
    Convert points to cartesian coordinates

    (dict) -> return X, Y, Z
    """
    Xs = np.array([])
    Ys = np.array([])
    #Zs = np.array([])

    for key, (x, y, z) in trans.items():
        Xs = np.append(Xs, x)
        Ys = np.append(Ys, y)
        # Zs = np.append(Zs, z)

    return Xs, Ys  #, Zs


def poly_area((Xs, Ys)):
    """
    Implementation of Shoelace formula

    (X, Y, Z) -> return (float)
    """
    return 0.5 * np.abs(np.dot(Xs, np.roll(Ys, 1)) - np.dot(Ys, np.roll(Xs, 1)))


def measure_grasp_polygon_area(trans):
    """
    Calculates polygon area

    () -> return (float)
    """
    return poly_area(convert_points_to_Xs_and_Ys(trans))

"""
Computes the angles of polygon at each finger-tip
"""
def measure_grasp_polygon_angles(finger_tips):

    n = len(finger_tips)
    print n

    ideal_angle = (180 * (n-2) / n)

    summation = 2

    measure = summation/((n-2) * (180 - ideal_angle) * 2*ideal_angle)

    return measure


# if __name__ == '__main__':
#
#     from sr_grasp_stability.tf2_computation import TfComputator
#
#     TF_comp = TfComputator()
#
#     finger_tips = False
#
#     while not finger_tips:
#         finger_tips, exc = TF_comp.get_finger_tips('world')
#         print finger_tips
#     print finger_tips
#
#     measure_grasp_polygon_angles(finger_tips)