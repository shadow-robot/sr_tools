#!/usr/bin/env python3

# Copyright 2020-2023 Shadow Robot Company Ltd.
#
# This program is free software: you can redistribute it and/or modify it
# under the terms of the GNU General Public License as published by the Free
# Software Foundation version 2 of the License.
#
# This program is distributed in the hope that it will be useful, but WITHOUT
# ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
# FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
# more details.
#
# You should have received a copy of the GNU General Public License along
# with this program. If not, see <http://www.gnu.org/licenses/>.
from builtins import round
import rospy
from sr_hand_health_report.sr_hand_health_report_check import (SrHealthReportCheck,
                                                               Finger,
                                                               SENSOR_CUTOUT_THRESHOLD,
                                                               NR_OF_BITS_NOISE_WARNING)
import numpy as np

SW_LIMITS_FOR_JOINTS = {"wrj1": -0.785, "thj5": 1.047}


class MonotonicityCheck(SrHealthReportCheck):

    PASSED_THRESHOLDS = {'is_monotonic': True, 'higher_raw_sensor_value': 4090, 'lower_raw_sensor_value': 5}

    def __init__(self, hand_side, fingers_to_test):
        """
            Initialize the MonotonicityCheck object
            @param hand_side: String indicating the side
            @param fingers_to_test: List of finger prefixes to test
        """
        super().__init__(hand_side, fingers_to_test)
        self._name = "Monotonicity"
        self._is_joint_monotonous = True
        self._dict_of_monotonic_joints = {}
        self._publishing_rate = rospy.Rate(50)  # 50 Hz
        self._older_raw_sensor_value = 0
        self._previous_difference = 0
        self._pwm_command = 250
        self._check_duration = rospy.Duration(4.0)
        self._first_end_stop_sensor_value = None
        self._second_end_stop_sensor_value = None
        self._result = {'monotonicity': {}}

    def run_check(self):
        """
            Runs the check for all fingers
        """
        self._result = {'monotonicity': {}}
        if self._stopped_execution:
            self._stopped_execution = False
            return
        rospy.loginfo("Running Monotonicity Check")
        result = dict(self._result)

        self.move_fingers_to_start_position()
        self.switch_controller_mode("effort")

        Finger.PREFIXES = ("FF", 'MF', 'LF', 'RF', 'TH', 'WR')
        self.fingers_to_check.sort(reverse=False, key=lambda x: x._get_sorting_value())

        for finger in self.fingers_to_check:
            self._run_check_per_finger(finger)
            if self._stopped_execution:
                self._stopped_execution = False
                return

        result["monotonicity"] = dict(self._dict_of_monotonic_joints)
        self.switch_controller_mode("position")
        self._result = result
        self._stopped_execution = True
        return

    def _run_check_per_finger(self, finger):
        """
            Runs the check for the provided finger
            @param: Finger object to run the check on.
        """
        for joint in finger.joints_dict.values():
            if finger.finger_name == "wr":
                command = 350
            else:
                command = 250
            self._run_check_per_joint(joint, command, finger)
            if self._stopped_execution:
                break

    def _run_check_per_joint(self, joint, command, finger):
        """
            Runs the check for the provided finger
            @param: Finger object to run the check on.
            @param: int value of PWM command to be sent
            @param: Joint object to run the check on.
        """
        rospy.loginfo("Analyzing joint {}".format(joint.joint_name))
        joint_name = finger.finger_name + joint.joint_index
        extend_command = self.command_sign_map[joint_name]*command
        flex_command = -extend_command

        if finger.finger_name == "wr":
            if joint.joint_index == "j1":
                extend_command = -self.command_sign_map[joint_name]*600
                flex_command = extend_command - 900

        self._older_raw_sensor_value = 0
        self._previous_difference = 0
        self._is_joint_monotonous = True
        end_reached = False
        is_joint_monotonous = True

        time = rospy.Time.now() + self._check_duration
        while rospy.Time.now() < time and not rospy.is_shutdown():
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
                self._first_end_stop_sensor_value = MonotonicityCheck.get_raw_sensor_value(joint.get_raw_sensor_data())
            if self._stopped_execution:
                return

        self._second_end_stop_sensor_value = MonotonicityCheck.get_raw_sensor_value(joint.get_raw_sensor_data())

        higher_value, lower_value = MonotonicityCheck.check_sensor_range(self._first_end_stop_sensor_value,
                                                                         self._second_end_stop_sensor_value)

        self._add_result_to_dict(joint.joint_name, higher_value, lower_value)
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

    def _add_result_to_dict(self, joint_name, higher_value, lower_value):
        """
            Add the results to the result dicttionary
            @param: str joint name
            @param: float higher range value
            @param: float lower range value
        """
        self._dict_of_monotonic_joints[joint_name] = {}
        self._dict_of_monotonic_joints[joint_name]["is_monotonic"] = self._is_joint_monotonous
        self._dict_of_monotonic_joints[joint_name]["higher_raw_sensor_value"] = higher_value
        self._dict_of_monotonic_joints[joint_name]["lower_raw_sensor_value"] = lower_value

    def _check_monotonicity(self, joint):
        """
            Checks if the joint is monotonuous
            @param: Joint object to be tested
            @return: bool
        """
        if self._older_raw_sensor_value == 0:
            self._older_raw_sensor_value = MonotonicityCheck.get_raw_sensor_value(joint.get_raw_sensor_data())

        difference_between_raw_data = (MonotonicityCheck.get_raw_sensor_value(joint.get_raw_sensor_data()) -
                                       self._older_raw_sensor_value)
        self._older_raw_sensor_value = MonotonicityCheck.get_raw_sensor_value(joint.get_raw_sensor_data())
        if abs(difference_between_raw_data) <= SENSOR_CUTOUT_THRESHOLD:
            if abs(difference_between_raw_data) > NR_OF_BITS_NOISE_WARNING:
                if abs(self._previous_difference) > NR_OF_BITS_NOISE_WARNING:
                    if np.sign(difference_between_raw_data) != 0 and np.sign(self._previous_difference) != 0:
                        if np.sign(difference_between_raw_data) != np.sign(self._previous_difference):
                            rospy.logwarn("Unmonotonic behaviour detected")
                            self._previous_difference = difference_between_raw_data
                            return False
                    self._previous_difference = difference_between_raw_data
        self._previous_difference = difference_between_raw_data
        return True

    def _check_joint_limit(self, joint):
        """
            This function check the joint position to avoid intense stress on WR1 and TH5,
            which would be caused by executing constant PWM command
        """
        limit_reached = False
        if joint.joint_name == self._hand_prefix + "_wrj1":
            if joint.get_current_position() - SW_LIMITS_FOR_JOINTS["wrj1"] < 0.01:
                limit_reached = True
        elif joint.joint_name == self._hand_prefix + "_thj5":
            if abs(abs(joint.get_current_position()) - SW_LIMITS_FOR_JOINTS["thj5"]) < 0.01:
                limit_reached = True
        return limit_reached

    def has_passed(self):
        """
            Checks if the test execution result passed
            @return bool check passed
        """
        passed = True
        for joint_result in self._result['monotonicity'].keys():
            for key in self._result['monotonicity'][list(self._result['monotonicity'].keys())[0]]:
                if not self.has_single_passed(key, self._result['monotonicity'][joint_result][key]):
                    passed = False
                    break
        return passed and bool(self._result["monotonicity"])

    def has_single_passed(self, name, value):
        """
            Checks if the single test execution result passed
            @param name: name of the test
            @param value: value to be compared with the thresholds
            @return bool check passed
        """
        output = False
        if name == 'is_monotonic':
            output = MonotonicityCheck.PASSED_THRESHOLDS['is_monotonic'] == value
        elif name == 'higher_raw_sensor_value':
            output = MonotonicityCheck.PASSED_THRESHOLDS['higher_raw_sensor_value'] > value
        elif name == 'lower_raw_sensor_value':
            output = MonotonicityCheck.PASSED_THRESHOLDS['lower_raw_sensor_value'] < value
        return output

    @staticmethod
    def check_sensor_range(first_sensor_value, second_sensor_value):
        """
            Checks the sensors range and return them in high to low order.
            @param: float first_sensor_value
            @param: float second_sensor_value
        """
        if first_sensor_value > second_sensor_value:
            higher_value = first_sensor_value
            lower_value = second_sensor_value
        else:
            higher_value = second_sensor_value
            lower_value = first_sensor_value
        return higher_value, lower_value

    @staticmethod
    def get_raw_sensor_value(data):
        """
            Checks the sensors range and return them in high to low order.
            @param: list of data
            @return: float average of the data
        """
        return sum(data) / len(data)
