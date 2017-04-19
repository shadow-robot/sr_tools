#!/usr/bin/env python

import rospy
import tf2_ros


class TfComputator:
    """
    Hand tf transformation computation
    """

    def __init__(self):

        self.list_of_fingertip_information = []
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)

    def get_fingertips(self, list_of_fingertips, reference_frame='world', individual_tf_timeout=0.5):
        """
        :param list_of_fingertips: A list of the fingertip frames which transforms will be measured to
            from the reference_frame
        :param reference_frame: Frame in which transforms will be generated from
        :return fingertip_transforms: Dictionary of transform vectors from reference_frame to fingertips
        """
        self.list_of_fingertip_information = []

        for fingertip_in in list_of_fingertips:
            fingertip = {'name': fingertip_in, 'transform': None}
            start_time = rospy.get_time()
            while not rospy.is_shutdown():
                try:
                    fingertip['transform'] = self.tfBuffer.lookup_transform(reference_frame, fingertip_in,
                                                                            rospy.Time()).transform
                    break
                except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                    if rospy.get_time() - start_time > individual_tf_timeout:
                        rospy.logerr("tf frames not received for %s - Time out occurred" % fingertip['name'])
                        break
                    rospy.sleep(.01)

            self.list_of_fingertip_information.append(fingertip)

        return self.list_of_fingertip_information
