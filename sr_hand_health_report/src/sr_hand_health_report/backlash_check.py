#!/usr/bin/env python3

# Copyright 2022-2023 Shadow Robot Company Ltd.
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

import time
import math
import rospy
from sr_hand_health_report.sr_hand_health_report_check import SrHealthReportCheck
import numpy as np
from urdf_parser_py.urdf import URDF


class BacklashCheck(SrHealthReportCheck):

    TEST_COUNT = 10
    THRESHOLD = 0.01
    FINGER_PWM = -250
    WIGGLING_TIME = 2
    FINGERS = ("TH", "FF", "MF", "RF", "LF")
    PASSED_THRESHOLDS = {'std': 0.015, 'avg': 0.01}

    """
        Initialize the BacklashCheck object
        @param hand_side: String indicating the side
        @param fingers_to_test: List of finger prefixes to test
    """
    def __init__(self, hand_side, fingers_to_test):
        super().__init__(hand_side, fingers_to_test)
        self._side_sign_map = {"ff": -1, "mf": -1, "rf": 1, "lf": 1, "th": 1, "wr": 1}
        self._name = "backlash"
        self._result = {'backlash': {}}
        self.joint_limits = {}
        self._set_joint_limits()

    """
        Sets the joint limits from the URDF
    """
    def _set_joint_limits(self):
        joint_list = [joint_name.lower() for joint_name in self._joint_msg.name]
        for joint in URDF.from_parameter_server().joints:
            if joint.name.lower() in joint_list:
                self.joint_limits[joint.name.lower()] = joint.limit

    """
        Moves tested fingers into 0 degree joint anglees in position control mode.
    """
    def move_fingers_to_start_position(self):
        self.switch_controller_mode('position')
        for finger in self.fingers_to_check:
            finger.move_finger(0, 'position')
            rospy.logwarn(f"Moving to start {finger.finger_name}")

    """
        Runs the backlash check for all selected fingers and assings the result into the _result variable.
    """
    def run_check(self):

        if self._stopped_execution:
            self._stopped_execution = False
            return

        self.move_fingers_to_start_position()

        self.switch_controller_mode("position")
        for finger_object in self.fingers_to_check:
            self.move_finger_to_side(finger_object, 'right')

        for finger_object in self.fingers_to_check:
            finger_object.move_finger(0, 'position')
            self.switch_controller_mode("effort")

            for joint in finger_object.joints_dict.values():
                rospy.logwarn(f"Running check for {joint.joint_name}")
                if finger_object.finger_name in ('ff', 'mf', 'rf', 'lf') and joint.joint_index != "j1":
                    # rospy.logerr(f"Wiggling {finger_object.finger_name} {joint.joint_index}")
                    joint_result = self.wiggle_joint(joint)
                    self._result['backlash'][joint.joint_name] = joint_result
                elif finger_object.finger_name in ('th', 'wr'):
                    joint_result = self.wiggle_joint(joint)
                    self._result['backlash'][joint.joint_name] = joint_result

                if self._stopped_execution:
                    break
            if self._stopped_execution:
                return

            self.switch_controller_mode("position")
            self.move_finger_to_side(finger_object, 'left')

        self._stopped_execution = False

        return

    """
        Moves the finger to the left/right joint limit of J4
        @param finger_object: Finger object indicating which finger to move
        @param side: String defining the side, 'right' or 'left'
    """
    def move_finger_to_side(self, finger_object, side):
        angle = math.radians(-20) if side == 'right' else math.radians(20)
        angle *= self._side_sign_map[finger_object.finger_name]
        if "J4" in finger_object.joints_dict:
            finger_object.joints_dict['J4'].move_joint(angle, 'position')

    """
        Moves the joint repeatably by switching the PWM sign and sending the commands to the effort controller.
        The result is being save into the _results variable and contains the average and standard deviation.
        @param joint: Joint object indicating which joint to 'wiggle'
    """
    def wiggle_joint(self, joint):
        test_times = []
        total_time = None

        test_start_time = rospy.get_rostime()
        timeout_condition = True

        while len(test_times) < self.TEST_COUNT and timeout_condition and not rospy.is_shutdown():

            success = True
            test_time = rospy.get_rostime()
            difference = 0
            starting_position = joint.get_current_position()
            start_time = rospy.get_rostime()
            joint.move_joint(-self.FINGER_PWM, 'effort')
            current_time = 0
            while abs(difference) < self.THRESHOLD and not rospy.is_shutdown():
                difference = joint.get_current_position() - starting_position
                current_time = rospy.get_rostime() - start_time
                time.sleep(0.01)
                if current_time > rospy.Duration.from_sec(self.WIGGLING_TIME):
                    test_time += rospy.Duration.from_sec(self.WIGGLING_TIME)
                    success = False
                    break

            difference = 0
            starting_position = joint.get_current_position()
            start_time = rospy.get_rostime()
            joint.move_joint(self.FINGER_PWM, 'effort')
            current_time = 0
            while abs(difference) < self.THRESHOLD and not rospy.is_shutdown():
                difference = joint.get_current_position() - starting_position
                current_time = rospy.get_rostime() - start_time
                time.sleep(0.01)
                if current_time > rospy.Duration.from_sec(self.WIGGLING_TIME):
                    test_time += rospy.Duration.from_sec(self.WIGGLING_TIME)
                    success = False
                    break

            timeout_condition = rospy.get_rostime() - test_start_time < rospy.Duration.from_sec(3 * self.WIGGLING_TIME)

            if success:
                test_time = rospy.get_rostime() - test_time
                test_times.append(test_time)

                if not total_time:
                    total_time = test_time
                else:
                    total_time += test_time

        joint.move_joint(0, 'effort')

        result = {}
        times = [time_entry.nsecs/10e9 for time_entry in test_times]
        # Casting the numpy methods to float, as the yaml package cannot handle it automatically.
        result['std'] = float(np.std(times))
        result['avg'] = float(np.mean(times))
        return result

    '''
    def _debounce(self, joint, timeout=1):
        if joint.get_current_position() < self.joint_limits[joint.joint_name].lower + math.radians(1):
            joint.move_joint(-self.FINGER_PWM, 'effort')
            rospy.sleep(timeout)
            joint.move_joint(0, 'effort')
        if joint.get_current_position() > self.joint_limits[joint.joint_name].upper - math.radians(1):
            joint.move_joint(self.FINGER_PWM, 'effort')
            rospy.sleep(timeout)
            joint.move_joint(0, 'effort')
    '''

    """
        Checks if the test execution result passed
        @return bool check passed
    """
    def has_passed(self):
        # to be edited after value confirmation with Luke and production
        passed = True

        for joint_name in self._result['backlash'].keys():
            joint_data = self._result['backlash'][joint_name]
            for key in self.PASSED_THRESHOLDS:
                if not self.has_single_passed(key, joint_data[key]):
                    passed = False
                    break
        return passed and bool(self._result["backlash"])

    """
        Checks if the single test execution result passed
        @param name: name of the test
        @param value: value to be compared with the thresholds
        @return bool check passed
    """
    def has_single_passed(self, name, value):
        return value < self.PASSED_THRESHOLDS[name]


if __name__ == '__main__':
    rospy.init_node("sr_backlash")

    backlash = BacklashCheck('right', BacklashCheck.FINGERS)
    backlash.run_check()

    for finger_element in backlash.fingers_to_check:
        finger_element.move_finger(0, 'position')
