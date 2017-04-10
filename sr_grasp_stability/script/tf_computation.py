#!/usr/bin/env python

import rospy
import tf

class TfComputator:
    """
    Hand tf transformation computation
    """

    def __init__(self):

        self.trans = {}
        self. rot = {}
        self.tf_listener = tf.TransformListener()

    def get_finger_tips(self):
        """
        Get fingertip transformations
        () -> return (dict, dict)
        """
        # finger-tip frames:
        # '/rh_rftip', '/rh_lftip', '/rh_mftip', '/rh_thtip', '/rh_fftip',

        # Get transforms from forearm to distal #
        try:
            (self.trans['rh_ffdistal'], self.rot['rh_ffdistal']) = self.tf_listener.lookupTransform(
                '/rh_ffdistal', '/rh_forearm', rospy.Time(0))

            (self.trans['rh_lfdistal'], self.rot['rh_lfdistal']) = self.tf_listener.lookupTransform(
                '/rh_lfdistal', '/rh_forearm', rospy.Time(0))

            (self.trans['rh_mfdistal'], self.rot['rh_mfdistal']) = self.tf_listener.lookupTransform(
                '/rh_mfdistal', '/rh_forearm', rospy.Time(0))

            (self.trans['rh_rfdistal'], self.rot['rh_rfdistal']) = self.tf_listener.lookupTransform(
                '/rh_rfdistal', '/rh_forearm', rospy.Time(0))

            (self.trans['rh_thdistal'], self.rot['rh_thdistal']) = self.tf_listener.lookupTransform(
                '/rh_thdistal', '/rh_forearm', rospy.Time(0))

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.loginfo("Failed to get tf for grasp measurement computation")

        return self.trans, self.rot
