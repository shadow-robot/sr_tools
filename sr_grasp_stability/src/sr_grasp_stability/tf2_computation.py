#!/usr/bin/env python

import rospy
import math
import tf2_ros
import geometry_msgs.msg

class TfComputator:
    """
    Hand tf transformation computation
    """

    def __init__(self):

        self.trans = {}
        # self. rot = {}
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)

    def get_finger_tips(self):
        """
        Get fingertip transformations

        () -> return (dict, dict)
        """
        # finger-tip frames:
        # '/rh_rftip', '/rh_lftip', '/rh_mftip', '/rh_thtip', '/rh_fftip',

        exc = False
        # Get transforms from forearm to distal #
        try:
            transform = self.tfBuffer.lookup_transform('world', 'rh_fftip', rospy.Time()).transform.translation
            self.trans['rh_fftip'] = (transform.x, transform.y, transform.z)

            transform = self.tfBuffer.lookup_transform('world', 'rh_mftip', rospy.Time()).transform.translation
            self.trans['rh_mftip'] = (transform.x, transform.y, transform.z)

            transform = self.tfBuffer.lookup_transform('world', 'rh_rftip', rospy.Time()).transform.translation
            self.trans['rh_rftip'] = (transform.x, transform.y, transform.z)

            transform = self.tfBuffer.lookup_transform('world', 'rh_lftip', rospy.Time()).transform.translation
            self.trans['rh_lftip'] = (transform.x, transform.y, transform.z)

            transform = self.tfBuffer.lookup_transform('world', 'rh_thtip', rospy.Time()).transform.translation
            self.trans['rh_thtip'] = (transform.x, transform.y, transform.z)

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.loginfo("Failed to get tf for grasp measurement computation")
            exc = True

        return self.trans, exc
