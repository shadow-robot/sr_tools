#!/usr/bin/env python

# Copyright (C) 2019 Shadow Robot Company Ltd - All Rights Reserved. Proprietary and Confidential.
# Unauthorized copying of the content in this file, via any medium is strictly prohibited.

import rospy
from sr_hand_health_report_check import SrHealthReportCheck
from std_msgs.msg import Float64
from multiprocessing import Process
import numpy as np


class MonotonicityCheck(SrHealthReportCheck):
    def __init__(self):
        super(MonotonicityCheck, self).__init__("lh")
        self._is_joint_monotonous = True
        self._dict_of_monotonic_joints = {}
        self._count_time = 0
        self._publishing_rate = rospy.Rate(50)
        self._older_raw_sensor_value = 0
        self._previous_difference = 0
        self._pwm_command = 250

    def run_check(self):
        self.reset_robot_to_home_position()
        rospy.sleep(1.0)

        result = {"monotonicity_check" : []}
        rospy.loginfo("Running Monotonicity Test")
        self.switch_controller_mode("effort")
        for finger in self.fingers_to_check:
            if finger.finger_name is 'FF':
                for joint in finger.joints:
                    if joint.joint_name == "lh_ffj4":
                        self.drive_joint_to_position(finger.joints[2], 0.785)
                    rospy.loginfo("Analyzing joint {}".format(joint.joint_name))
                    self._is_joint_monotonous = True
                    self._count_time = 0
                    end_reached = False
                    while self._count_time < 200:
                        joint.move_joint(self._pwm_command, "effort")
                        is_joint_monotonous = self.check_monotonicity(joint)
                        if is_joint_monotonous == False:
                            self._is_joint_monotonous = False
                        self._count_time += 1
                        self._publishing_rate.sleep()
                        if self._count_time == 200 and end_reached == False:
                            self._count_time = 0
                            self._pwm_command = - self._pwm_command
                            end_reached = True
                    self._dict_of_monotonic_joints[joint.joint_name] = self._is_joint_monotonous
                    self.drive_joint_to_position(joint, 0.0)
                    if joint.joint_name == "lh_ffj4":
                        self.drive_joint_to_position(finger.joints[2], 0)
        result["monotonicity_check"].append(self._dict_of_monotonic_joints)
        return result

    def check_monotonicity(self, joint):
        if self._older_raw_sensor_value == 0:
            self._older_raw_sensor_value = joint._raw_sensor_data

        difference_between_raw_data = joint._raw_sensor_data - self._older_raw_sensor_value
        self._older_raw_sensor_value = joint._raw_sensor_data
        if abs(difference_between_raw_data) <= 1:
            self._previous_difference = difference_between_raw_data
        else:
            if np.sign(difference_between_raw_data) != 0 and np.sign(self._previous_difference) != 0:
                if np.sign(difference_between_raw_data) != np.sign(self._previous_difference):
                    rospy.logwarn("derivative changed sign")
                    return False
            self._previous_difference = difference_between_raw_data
        return True

    def drive_joint_to_position(self, joint, command):
        self.switch_controller_mode("position")
        joint.move_joint(command, "position")
        self.switch_controller_mode("effort")
        self._count_time = 0
