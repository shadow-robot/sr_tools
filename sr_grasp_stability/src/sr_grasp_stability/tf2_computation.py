#!/usr/bin/env python

import rospy
import tf2_ros


class TfComputator:
    """
    Hand tf transformation computation
    """

    def __init__(self):

        self.trans = {}
        # self. rot = {}
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)

    def get_fingertips(self, ref_frame):
        """
        Get fingertip transformations

        () -> return (dict, dict)
        """

        exc = False
        # Get transforms from reference frame (ref_frame) to fingertips
        try:
            transform = self.tfBuffer.lookup_transform(ref_frame, 'rh_fftip', rospy.Time()).transform.translation
            self.trans['rh_fftip'] = (transform.x, transform.y, transform.z)

            transform = self.tfBuffer.lookup_transform(ref_frame, 'rh_mftip', rospy.Time()).transform.translation
            self.trans['rh_mftip'] = (transform.x, transform.y, transform.z)

            transform = self.tfBuffer.lookup_transform(ref_frame, 'rh_rftip', rospy.Time()).transform.translation
            self.trans['rh_rftip'] = (transform.x, transform.y, transform.z)

            transform = self.tfBuffer.lookup_transform(ref_frame, 'rh_lftip', rospy.Time()).transform.translation
            self.trans['rh_lftip'] = (transform.x, transform.y, transform.z)

            transform = self.tfBuffer.lookup_transform(ref_frame, 'rh_thtip', rospy.Time()).transform.translation
            self.trans['rh_thtip'] = (transform.x, transform.y, transform.z)

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.loginfo("Failed to get tf for grasp measurement computation")
            exc = True

        return self.trans, exc
