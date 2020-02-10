#!/usr/bin/env python

# Copyright (C) 2019 Shadow Robot Company Ltd - All Rights Reserved. Proprietary and Confidential.
# Unauthorized copying of the content in this file, via any medium is strictly prohibited.

import rospy
from sr_hand_health_report_check import SrHealthReportCheck, SENSOR_CUTOUT_THRESHOLD, NR_OF_BITS_NOISE_WARNING


class PositionSensorNoiseCheck(SrHealthReportCheck):
    def __init__(self, hand_side):
        super(PositionSensorNoiseCheck, self).__init__(hand_side)
        self._check_duration = rospy.Duration(3.0)
        self._shared_dict = dict()
        self._publishing_rate = rospy.Rate(200)

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

    def check_joint_raw_sensor_value(self, initial_raw_value, joint, dictionary):
        status = ""
        warning = False
        test_failed = False
        if joint.joint_name != self._hand_prefix + "_wrj1" and joint.joint_name != self._hand_prefix + "_thj5":
            initial_raw_value = initial_raw_value[-1:]
        for i, value in enumerate(initial_raw_value):
            time = rospy.Time.now() + self._check_duration
            while rospy.Time.now() < time and test_failed is not True:
                difference = joint._raw_sensor_data[i] - initial_raw_value[i]
                if abs(difference) <= SENSOR_CUTOUT_THRESHOLD:
                    if abs(difference) < NR_OF_BITS_NOISE_WARNING:
                        status = "{} bits noise - CHECK PASSED".format(abs(difference) + 1)
                    elif abs(difference) == NR_OF_BITS_NOISE_WARNING:
                        rospy.logwarn_throttle(1, "Found value with 3 bits diff")
                        status = "3 bits noise - WARNING"
                        test_failed = True
                    else:
                        rospy.logerr_throttle(1, "Found value with {} bits diff".format(abs(difference) + 1))
                        status = "{} bits noise - CHECK FAILED".format(abs(difference) + 1)
                        test_failed = True
                self._publishing_rate.sleep()
        print("Finished loop for {}".format(joint.joint_name))
        dictionary[joint.joint_name] = status
