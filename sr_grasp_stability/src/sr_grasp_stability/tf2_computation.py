#!/usr/bin/env python

import rospy
import tf2_ros


class TfComputator:
    """
    Hand tf transformation computation
    """

    def __init__(self):

        self.fingertip_transforms = {}
        # self. rot = {}
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)

    def get_fingertips(self, list_of_fingertips, reference_frame='world'):
        """

        :param list_of_fingertips: A list of the fingertip frames which transforms will be measured to
            from the reference_frame
        :param reference_frame: Frame in which transforms will be generated from
        :return:
        """

        # Hand E fingertip frames:
        # '/rh_fftip', '/rh_mftip', '/rh_rftip', '/rh_lftip', '/rh_thtip',

        exception_has_been_triggered = False

        for fingertip in list_of_fingertips:
            try:
                transform = self.tfBuffer.lookup_transform(reference_frame, fingertip,
                                                           rospy.Time()).transform.translation
                self.fingertip_transforms[fingertip] = (transform.x, transform.y, transform.z)

            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                rospy.loginfo("Failed to get tf for grasp measurement computation")
                exception_has_been_triggered = True

        # # Get transforms from reference frame to fingertips
        # try:
        #     transform = self.tfBuffer.lookup_transform(reference_frame, 'rh_fftip', rospy.Time()).transform.translation
        #     self.trans['rh_fftip'] = (transform.x, transform.y, transform.z)
        #
        #     transform = self.tfBuffer.lookup_transform(reference_frame, 'rh_mftip', rospy.Time()).transform.translation
        #     self.trans['rh_mftip'] = (transform.x, transform.y, transform.z)
        #
        #     transform = self.tfBuffer.lookup_transform(reference_frame, 'rh_rftip', rospy.Time()).transform.translation
        #     self.trans['rh_rftip'] = (transform.x, transform.y, transform.z)
        #
        #     transform = self.tfBuffer.lookup_transform(reference_frame, 'rh_lftip', rospy.Time()).transform.translation
        #     self.trans['rh_lftip'] = (transform.x, transform.y, transform.z)
        #
        #     transform = self.tfBuffer.lookup_transform(reference_frame, 'rh_thtip', rospy.Time()).transform.translation
        #     self.trans['rh_thtip'] = (transform.x, transform.y, transform.z)
        #
        # except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        #     rospy.loginfo("Failed to get tf for grasp measurement computation")
        #     exception_has_been_triggered = True

        return self.fingertip_transforms, exception_has_been_triggered
