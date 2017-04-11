#!/usr/bin/env python

import numpy as np

class Polygon:
    """
    Computes the polygon area of the grasp
    """

    def __init__(self):
        pub = rospy.Publisher('grasp_quality_measure', Polygon, queue_size=10)

    def convert_points_to_Xs_and_Ys(self, trans):
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
            #Zs = np.append(Zs, z)

        return Xs, Ys  #, Zs

    def poly_area(self, (Xs, Ys)):
        """
        Implementation of Shoelace formula

        (X, Y, Z) -> return (float)
        """
        return 0.5 * np.abs(np.dot(Xs, np.roll(Ys, 1)) - np.dot(Ys, np.roll(Xs, 1)))

    def measure_grasp_polygon_area(self, trans):
        """
        Calculates polygon area

        () -> return (float)
        """
        return self.poly_area(self.convert_points_to_Xs_and_Ys(trans))
