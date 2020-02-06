#!/usr/bin/env python

# Copyright (C) 2019 Shadow Robot Company Ltd - All Rights Reserved. Proprietary and Confidential.
# Unauthorized copying of the content in this file, via any medium is strictly prohibited.

import rospy
from sr_hand_health_report_check import SrHealthReportCheck, SENSOR_CUTOUT_THRESHOLD, NR_OF_BITS_NOISE_WARNING


class PositionSensorNoiseCheck(SrHealthReportCheck):
    def __init__(self, hand_side):
        super(PositionSensorNoiseCheck, self).__init__(hand_side)
        self._check_duration = rospy.Duration(3.0)
        self._shared_dict = self.manager.dict()

    def run_check(self):
        self.reset_robot_to_home_position()
        rospy.sleep(1.0)
        result = {"position_sensor_noise_check" : []}
        rospy.loginfo("Running Position Sensor Noise Check")

        for finger in self.fingers_to_check:
            rospy.loginfo("collecting and analyzing data for FINGER {}".format(finger.finger_name))
            for joint in finger.joints_dict.values():
                rospy.loginfo("collecting and analyzing data for JOINT {}".format(joint.joint_name))
                self._initial_raw_value = joint._raw_sensor_data
                self.check_joint_raw_sensor_value(self._initial_raw_value, joint, self._shared_dict)
        result["position_sensor_noise_check"].append(dict(self._shared_dict))
        rospy.loginfo("Position Sensor Noise Check finished, exporting results")
        return result

    def check_joint_raw_sensor_value(self, initial_value, joint, dictionary):
        time = rospy.Time.now() + self._check_duration
        status = ""
        warning = False
        test_failed = False
        while rospy.Time.now() < time and test_failed is not True:
            difference = joint._raw_sensor_data - initial_value
            if abs(difference) <= SENSOR_CUTOUT_THRESHOLD:
                if abs(difference) < NR_OF_BITS_NOISE_WARNING:
                    status = "{} bits noise - CHECK PASSED".format(difference)
                elif abs(difference) == NR_OF_BITS_NOISE_WARNING:
                    rospy.logwarn_throttle(1, "Found value with 3 bits diff")
                    status = "3 bits noise - WARNING"
                    test_failed = True
                else:
                    rospy.logerr_throttle(1, "Found value with {} bits diff".format(difference))
                    status = "{} bits noise - CHECK FAILED".format(difference)
                    test_failed = True
        print("Finished loop for {}".format(joint.joint_name))
        dictionary[joint.joint_name] = status
