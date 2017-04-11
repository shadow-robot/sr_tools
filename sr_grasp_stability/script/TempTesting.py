#!/usr/bin/env python

# import roslib
# roslib.load_manifest('learning_tf')

import rospy
import tf

import numpy as np

# Terminal command to run:
# rosrun sr_grasp_stability tf_computation.py


class TfComputator:

    def __init__(self):

        self.trans = {} # initialize trans dict
        self.tf_listener = tf.TransformListener() # initialize listener

    def get_finger_tips(self):
        # finger-tip frames:
        # '/rh_rftip', '/rh_lftip', '/rh_mftip', '/rh_thtip', '/rh_fftip',

        # Get transforms from forearm to distal #
        try:
            (self.trans['rh_ffdistal'], rot_rh_rftip) = self.tf_listener.lookupTransform(
                '/rh_ffdistal', '/rh_forearm', rospy.Time(0))
            (self.trans['rh_lfdistal'], rot_rh_rftip) = self.tf_listener.lookupTransform(
                '/rh_lfdistal', '/rh_forearm', rospy.Time(0))
            (self.trans['rh_mfdistal'], rot_rh_rftip) = self.tf_listener.lookupTransform(
                '/rh_mfdistal', '/rh_forearm', rospy.Time(0))
            (self.trans['rh_rfdistal'], rot_rh_lftip) = self.tf_listener.lookupTransform(
                '/rh_rfdistal', '/rh_forearm', rospy.Time(0))

            (self.trans['rh_thdistal'], rot_rh_thtip) = self.tf_listener.lookupTransform(
                '/rh_thdistal', '/rh_forearm', rospy.Time(0))

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.loginfo("Failed to get tf for grasp measurement computation")
            # continue

    def convert_points_to_Xs_and_Ys(self):
        """
        Convert point to cartesian coordinates
        """
        Xs = np.array([])
        Ys = np.array([])
        Zs = np.array([])

        for key, (x, y, z) in self.trans.items():
            Xs = np.append(Xs, x)
            Ys = np.append(Ys, y)
            Zs = np.append(Zs, z)

        return Xs, Ys



    def poly_area(self, (Xs, Ys)):
        """
        Implementation of Shoelace formula
        """
        return 0.5 * np.abs(np.dot(Xs, np.roll(Ys, 1)) - np.dot(Ys, np.roll(Xs, 1)))

    def measure_grasp_polygon_area(self):
        """
        Calculates polugon area
        """
        self.get_finger_tips()
        return self.poly_area(self.convert_points_to_Xs_and_Ys())

if __name__ == '__main__':

    rospy.init_node('sr_grasp_stability')

    tf_computator = TfComputator()

    rate = rospy.Rate(10.0)  # Hz

    while not rospy.is_shutdown():

        # print 'The grasp 2d-polygon area is ', tf_computator.measure_grasp_polygon_area()
        rospy.loginfo("The grasp 2d-polygon area is %s", tf_computator.measure_grasp_polygon_area())

        rate.sleep()