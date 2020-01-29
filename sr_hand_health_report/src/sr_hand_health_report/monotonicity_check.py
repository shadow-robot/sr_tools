#!/usr/bin/env python

# Copyright (C) 2019 Shadow Robot Company Ltd - All Rights Reserved. Proprietary and Confidential.
# Unauthorized copying of the content in this file, via any medium is strictly prohibited.

import rospy
from sr_hand_health_report_check import SrHealthReportCheck
from std_msgs.msg import Float64
from multiprocessing import Process


class MonotonicityCheck(SrHealthReportCheck):
    def __init__(self):
        super(MonotonicityCheck, self).__init__("lh")

    def run_check(self):
        for finger in self.fingers_to_check:
            if finger.finger_name is not 'TH' and finger.finger_name is not 'LF':
                r = rospy.Rate(10)
                for joint in finger.joints:
                    while not joint.end_stop_reached_flag:
                        # adjust command motor polarization +/-
                        joint.move_joint(300.0)
                        r.sleep()
        return True

    def check_monotonicity(self):

        #if sensor_data is increasing all the time
        #return monotous
        pass


    def export_results(self):
        pass
