#!/usr/bin/env python

import numpy as np


def get_centre_of_grasp(fingertips):
    """
    Calculate the centre of the grasp
    :param fingertips: Dictionary which includes fingers
    :return (x, y, z): Coordinates for the centre of the fingertips position. AKA the average fingertip position.
    """
    num_of_fingertips = len(fingertips)
    sum_x = 0.0
    sum_y = 0.0
    sum_z = 0.0

    for fingertip in fingertips:
        translation = fingertip['transform'].translation
        sum_x += translation.x
        sum_y += translation.y
        sum_z += translation.z

    return (sum_x/num_of_fingertips), (sum_y/num_of_fingertips), (sum_z/num_of_fingertips)


def find_triangle_area(a, b, c):
    """ Calculates the area of a triangle given the points at its corners """
    ab = a - b
    ac = a - c

    cross_product = np.cross(ab, ac)
    triangle_area = np.linalg.norm(cross_product) / 2

    return triangle_area


def poly_3d_area(fingertips):
    """
    Measures the area of the polygon between the fingertips. This area is broken down into triangles for which the area
    is obtained and then recombined to give the area of the polygon.
    For more information see: https://medium.com/@ugocupcic/how-to-tell-if-my-robots-grasp-is-stable-7811fa3d16b8
    :param fingertips: List of dictionaries which contains fingertips and their relative location and orientation in
     an order that has been defined by the user.
    :return polygon_area: Area of the polygon
    """
    num_of_fingertips = len(fingertips)
    if num_of_fingertips < 3:
        return 0

    polygon_area = 0.0

    # Set first fingertip as common corner of all triangles
    common_triangle_corner = fingertips[0]['transform'].translation
    a = np.array([common_triangle_corner.x, common_triangle_corner.y, common_triangle_corner.z]) # 1st finger-tip

    for idx in xrange(2, num_of_fingertips):

        b = np.array([fingertips[idx - 1]['transform'].translation.x, fingertips[idx - 1]['transform'].translation.y,
                      fingertips[idx - 1]['transform'].translation.z])
        c = np.array([fingertips[idx]['transform'].translation.x, fingertips[idx]['transform'].translation.y,
                      fingertips[idx]['transform'].translation.z])

        polygon_area += find_triangle_area(a, b, c)

    return polygon_area


def find_cosine_angle(a, b, c):
    """ Finds angle at point b for triangle between points a, b and c """
    ba = a - b
    bc = c - b

    cosine_angle = np.dot(ba, bc) / (np.linalg.norm(ba) * np.linalg.norm(bc))
    angle = np.arccos(cosine_angle)

    return np.degrees(angle)


def measure_grasp_polygon_angles(fingertips):
    """
    Creates a measure of how good a grasp is by comparing the angles of a polygon between the fingertips with an ideal
    (equilateral) angle and max angle.
    For more information see: https://medium.com/@ugocupcic/how-to-tell-if-my-robots-grasp-is-stable-7811fa3d16b8
    :param fingertips: List of dictionaries which contains fingertips and their relative location and orientation in
     an order that has been defined by the user.
    :return measure: value
    """
    num_of_fingertips = len(fingertips)
    if num_of_fingertips < 3:
        return 0

    ideal_angle = (180 * (num_of_fingertips-2) / num_of_fingertips)
    max_angle = ((num_of_fingertips-2) * (180 - ideal_angle) * 2*ideal_angle)

    summation = 0.0

    for idx, fingertip in enumerate(fingertips):

        a = np.array([fingertips[idx - 1]['transform'].translation.x,
                      fingertips[idx - 1]['transform'].translation.y,
                      fingertips[idx - 1]['transform'].translation.z])

        b = np.array([fingertips[idx]['transform'].translation.x,
                      fingertips[idx]['transform'].translation.y,
                      fingertips[idx]['transform'].translation.z])

        if idx == (len(fingertips) - 1):  # if index is last element in list, set c to first
            c = np.array([fingertips[0]['transform'].translation.x,
                          fingertips[0]['transform'].translation.y,
                          fingertips[0]['transform'].translation.z])
        else:
            c = np.array([fingertips[idx + 1]['transform'].translation.x,
                          fingertips[idx + 1]['transform'].translation.y,
                          fingertips[idx + 1]['transform'].translation.z])

        angle = find_cosine_angle(a, b, c)
        summation += abs(angle - ideal_angle)

    measure = summation/max_angle

    return measure


def main():
    """ FOR TESTING LOCAL FUNCTIONS """
    a = np.array([0.0, 0.0, 1.0])
    b = np.array([2.0, 3.0, 1.0])
    c = np.array([4.0, 0.0, 1.0])
    print find_cosine_angle(a, b, c)

if __name__ == '__main__':
    main()
