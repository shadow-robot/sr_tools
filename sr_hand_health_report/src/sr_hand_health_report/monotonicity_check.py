#!/usr/bin/env python

# Copyright (C) 2019 Shadow Robot Company Ltd - All Rights Reserved. Proprietary and Confidential.
# Unauthorized copying of the content in this file, via any medium is strictly prohibited.

import rospy
from sr_hand_health_report_check import SrHealthReportCheck, SENSOR_CUTOUT_THRESHOLD, NR_OF_BITS_NOISE_WARNING
from std_msgs.msg import Float64
import numpy as np
import decimal

SW_LIMITS_FOR_JOINTS = {"wrj1": -0.785, "thj5": 1.047}


class MonotonicityCheck(SrHealthReportCheck):
    def __init__(self, hand_side, fingers_to_test):
        super(MonotonicityCheck, self).__init__(hand_side, fingers_to_test)
        self._is_joint_monotonous = True
        self._dict_of_monotonic_joints = {}
        self._publishing_rate = rospy.Rate(50)  # 50 Hz
        self._older_raw_sensor_value = 0
        self._previous_difference = 0
        self._pwm_command = 250
        self._check_duration = rospy.Duration(7.0)

    def run_check(self):
        result = {"monotonicity_check": []}
        rospy.loginfo("Running Monotonicity Check")
        self.switch_controller_mode("effort")

        for finger in self.fingers_to_check:
            self._run_check_per_finger(finger)

        result["monotonicity_check"].append(self._dict_of_monotonic_joints)
        rospy.loginfo("Monotonicity Check finished, exporting results")
        return result

    def _run_check_per_finger(self, finger):
        for joint in finger.joints_dict.values():
            if finger.finger_name == "wr":
                command = 350
            else:
                command = 250
            self._run_check_per_joint(joint, command, finger)

    def _run_check_per_joint(self, joint, command, finger):
        rospy.loginfo("Analyzing joint {}".format(joint.joint_name))
        joint_name = finger.finger_name + joint.joint_index
        extend_command = self.command_sign_map[joint_name]*command
        flex_command = -extend_command

        if finger.finger_name == "wr":
            if joint.joint_index == "j1":
                extend_command = -self.command_sign_map[joint_name]*600
                flex_command = extend_command - 750

        self._older_raw_sensor_value = 0
        self._previous_difference = 0
        self._is_joint_monotonous = True
        end_reached = False
        is_joint_monotonous = True
        joint_limit_reached = False

        time = rospy.Time.now() + self._check_duration
        while (rospy.Time.now() < time):
            if end_reached is False:
                joint.move_joint(extend_command, "effort")
            else:
                joint.move_joint(flex_command, "effort")
                is_joint_monotonous = self._check_monotonicity(joint)
            if is_joint_monotonous is False:
                self._is_joint_monotonous = False
            self._publishing_rate.sleep()
            if (round(rospy.Time.now().to_sec(), 1) == round(time.to_sec(), 1)) and end_reached is False:
                time = rospy.Time.now() + self._check_duration
                end_reached = True
                self._first_end_stop_sensor_value = self._get_raw_sensor_value(joint._raw_sensor_data)
                joint_limit_reached = False
        self._second_end_stop_sensor_value = self._get_raw_sensor_value(joint._raw_sensor_data)

        higher_value, lower_value = self._check_sensor_range(self._first_end_stop_sensor_value,
                                                             self._second_end_stop_sensor_value)

        self._add_result_to_dict(joint.joint_name, self._is_joint_monotonous, higher_value, lower_value)
        self._reset_joint_to_position(finger, joint, extend_command, flex_command)

    def _reset_joint_to_position(self, finger, joint, extend_command, flex_command):
        if finger.finger_name == "th":
            if joint.joint_index == "j1" or joint.joint_index == "j5":
                self.drive_joint_with_pwm(joint, flex_command, 2.0, self._publishing_rate)
            else:
                self.drive_joint_with_pwm(joint, extend_command, 2.0, self._publishing_rate)
        elif finger.finger_name == "wr":
            self.drive_joint_with_pwm(joint, extend_command, 1.0, self._publishing_rate)
        else:
            if joint.joint_index != "j3" and joint.joint_index != "j4":
                self.drive_joint_with_pwm(joint, extend_command, 5.0, self._publishing_rate)
            if joint.joint_index == "j4":
                self.drive_joint_with_pwm(joint, extend_command, 1.2, self._publishing_rate)
                self.drive_joint_with_pwm(finger.joints_dict["J3"],
                                          self.command_sign_map[finger.finger_name + "j3"] * 250, 3.0,
                                          self._publishing_rate)

    def _add_result_to_dict(self, joint_name, is_joint_monotonous, higher_value, lower_value):
        self._dict_of_monotonic_joints[joint_name] = {}
        self._dict_of_monotonic_joints[joint_name]["is_monotonic"] = self._is_joint_monotonous
        self._dict_of_monotonic_joints[joint_name]["higher_raw_sensor_value"] = higher_value
        self._dict_of_monotonic_joints[joint_name]["lower_raw_sensor_value"] = lower_value

    def _get_raw_sensor_value(self, data):
        return sum(data) / len(data)

    def _check_monotonicity(self, joint):
        if self._older_raw_sensor_value == 0:
            self._older_raw_sensor_value = self._get_raw_sensor_value(joint._raw_sensor_data)

        difference_between_raw_data = self._get_raw_sensor_value(joint._raw_sensor_data) - \
                                      self._older_raw_sensor_value
        self._older_raw_sensor_value = self._get_raw_sensor_value(joint._raw_sensor_data)
        if abs(difference_between_raw_data) <= SENSOR_CUTOUT_THRESHOLD:
            if (abs(difference_between_raw_data) and abs(self._previous_difference)) > NR_OF_BITS_NOISE_WARNING:
                if np.sign(difference_between_raw_data) != 0 and np.sign(self._previous_difference) != 0:
                    if np.sign(difference_between_raw_data) != np.sign(self._previous_difference):
                        rospy.logwarn("Unmonotonic behaviour detected")
                        self._previous_difference = difference_between_raw_data
                        return False
                self._previous_difference = difference_between_raw_data
        self._previous_difference = difference_between_raw_data
        return True

    def _check_sensor_range(self, first_sensor_value, second_sensor_value):
        """
        This function records the minimum and maximum range hit by the joint
        during the monotonicity check, this is collected to sanity check the
        sensor range
        """
        if first_sensor_value > second_sensor_value:
            higher_value = first_sensor_value
            lower_value = second_sensor_value
        else:
            higher_value = second_sensor_value
            lower_value = first_sensor_value
        return higher_value, lower_value

    def _check_joint_limit(self, joint):
        """
        This function check the joint position to avoid intense stress on WR1 and TH5,
        which would be caused by executing constant PWM command
        """
        limit_reached = False
        if joint.joint_name == self._hand_prefix + "_wrj1":
            if joint._current_position - SW_LIMITS_FOR_JOINTS["wrj1"] < 0.01:
                limit_reached = True
        elif joint.joint_name == self._hand_prefix + "_thj5":
            if abs(abs(joint._current_position) - SW_LIMITS_FOR_JOINTS["thj5"]) < 0.01:
                limit_reached = True
        return limit_reached
