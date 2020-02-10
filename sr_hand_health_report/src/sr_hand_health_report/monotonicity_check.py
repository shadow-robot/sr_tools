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
    def __init__(self, hand_side):
        super(MonotonicityCheck, self).__init__(hand_side)
        self._is_joint_monotonous = True
        self._dict_of_monotonic_joints = {}
        self._publishing_rate = rospy.Rate(50) # 50 Hz
        self._first_turn_older_raw_sensor_value = 0
        self._first_turn_previous_difference = 0
        self._second_turn_older_raw_sensor_value = 0
        self._second_turn_previous_difference = 0
        self._pwm_command = 250
        self._check_duration = rospy.Duration(5.0)

    def run_check(self):
        # self.reset_robot_to_home_position()
        # rospy.sleep(1.0)

        result = {"monotonicity_check" : []}
        rospy.loginfo("Running Monotonicity Check")
        self.switch_controller_mode("effort")

        for finger in self.fingers_to_check:
            if finger.finger_name == "FF":
                for joint in finger.joints_dict.values():
                    self._first_turn_older_raw_sensor_value = 0
                    self._first_turn_previous_difference = 0
                    self._second_turn_older_raw_sensor_value = 0
                    self._second_turn_previous_difference = 0

                    rospy.loginfo("Analyzing joint {}".format(joint.joint_name))

                    self._is_joint_monotonous = True
                    end_reached = False
                    is_joint_monotonous_second_turn = True
                    joint_limit_reached = False

                    # For joint 4 we want to move J3 to 90 degrees, in order to allow the full range of J4
                    # without hitting other fingers
                    if joint.joint_index == "j4":
                        self.drive_joint_to_position(finger.joints_dict["J3"], 1.57)

                    time = rospy.Time.now() + self._check_duration
                    while (rospy.Time.now() < time):
                        print("joint raw data: ", joint._raw_sensor_data)
                        if joint.joint_name[3:] in SW_LIMITS_FOR_JOINTS.keys():
                            joint_limit_reached = self._check_joint_limit(joint)
                        if joint_limit_reached is False:
                            joint.move_joint(self._pwm_command, "effort")
                        if end_reached is False:
                            is_joint_monotonous_first_turn = self._first_turn_check_monotonicity(joint)
                        else:
                            is_joint_monotonous_second_turn = self._second_turn_check_monotonicity(joint)
                        if is_joint_monotonous_first_turn is False or is_joint_monotonous_second_turn is False:
                            self._is_joint_monotonous = False
                        self._publishing_rate.sleep()
                        if (round(rospy.Time.now().to_sec(),1) == round(time.to_sec(),1)) and end_reached is False:
                            time = rospy.Time.now() + self._check_duration
                            self._pwm_command = - self._pwm_command
                            end_reached = True
                            self._first_end_stop_sensor_value = self._get_raw_sensor_value(joint._raw_sensor_data)
                            joint_limit_reached = False
                    self._second_end_stop_sensor_value = self._get_raw_sensor_value(joint._raw_sensor_data)

                    higher_value, lower_value = self._check_sensor_range(self._first_end_stop_sensor_value, self._second_end_stop_sensor_value)
                    self._dict_of_monotonic_joints[joint.joint_name] = {}
                    self._dict_of_monotonic_joints[joint.joint_name]["is_monotonic"] = self._is_joint_monotonous
                    self._dict_of_monotonic_joints[joint.joint_name]["higher_raw_sensor_value"] = higher_value
                    self._dict_of_monotonic_joints[joint.joint_name]["lower_raw_sensor_value"] = lower_value

                    self.drive_joint_to_position(joint, 0.0)

                    if joint.joint_index == "j4":
                        self.drive_joint_to_position(finger.joints_dict["J3"], 0.0)

        result["monotonicity_check"].append(self._dict_of_monotonic_joints)
        rospy.loginfo("Monotonicity Check finished, exporting results")
        return result

    def _get_raw_sensor_value(self, data):
        return sum(data) / len(data)

    def _first_turn_check_monotonicity(self, joint):
        if self._first_turn_older_raw_sensor_value == 0:
            self._first_turn_older_raw_sensor_value = self._get_raw_sensor_value(joint._raw_sensor_data)

        difference_between_raw_data = self._get_raw_sensor_value(joint._raw_sensor_data) - self._first_turn_older_raw_sensor_value
        self._first_turn_older_raw_sensor_value = self._get_raw_sensor_value(joint._raw_sensor_data)
        if abs(difference_between_raw_data) <= SENSOR_CUTOUT_THRESHOLD:
            if abs(difference_between_raw_data) > NR_OF_BITS_NOISE_WARNING:
                if np.sign(difference_between_raw_data) != 0 and np.sign(self._first_turn_previous_difference) != 0:
                    if np.sign(difference_between_raw_data) != np.sign(self._first_turn_previous_difference):
                        rospy.logwarn("Unmonotonic behaviour detected")
                        self._first_turn_previous_difference = difference_between_raw_data
                        return False
        self._first_turn_previous_difference = difference_between_raw_data
        return True

    def _second_turn_check_monotonicity(self, joint):
        if self._second_turn_older_raw_sensor_value == 0:
            self._second_turn_older_raw_sensor_value = self._get_raw_sensor_value(joint._raw_sensor_data)

        difference_between_raw_data = self._get_raw_sensor_value(joint._raw_sensor_data) - self._second_turn_older_raw_sensor_value
        self._second_turn_older_raw_sensor_value = self._get_raw_sensor_value(joint._raw_sensor_data)
        if abs(difference_between_raw_data) <= SENSOR_CUTOUT_THRESHOLD:
            if abs(difference_between_raw_data) > NR_OF_BITS_NOISE_WARNING:
                if np.sign(difference_between_raw_data) != 0 and np.sign(self._second_turn_previous_difference) != 0:
                    if np.sign(difference_between_raw_data) != np.sign(self._second_turn_previous_difference):
                        rospy.logwarn("Unmonotonic behaviour detected")
                        self._second_turn_previous_difference = difference_between_raw_data
                        return False
        self._second_turn_previous_difference = difference_between_raw_data
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
