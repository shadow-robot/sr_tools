#!/usr/bin/env python

import rospy
import tf

class tf_comp:
    def __init__(self):
        rospy.init_node('sr_grasp_stability')
        self.listener = tf.TransformListener()

    def listening(self):



        trans = {}



        try:
            (trans['rh_ffdistal'], rot_rh_rftip) = self.listener.lookupTransform('/rh_ffdistal', '/rh_forearm', rospy.Time(0))
            (trans['rh_ffdistal'], rot_rh_rftip) = self.listener.lookupTransform('/rh_lfdistal', '/rh_forearm', rospy.Time(0))
            (trans['rh_mfdistal'], rot_rh_rftip) = self.listener.lookupTransform('/rh_mfdistal', '/rh_forearm', rospy.Time(0))
            (trans['rh_ffdistal'], rot_rh_lftip) = self.listener.lookupTransform('/rh_rfdistal', '/rh_forearm', rospy.Time(0))

            (trans['rh_thdistal'], rot_rh_thtip) = self.listener.lookupTransform('/rh_thdistal', '/rh_forearm', rospy.Time(0))

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            print "exception"

        # print trans


        return trans
