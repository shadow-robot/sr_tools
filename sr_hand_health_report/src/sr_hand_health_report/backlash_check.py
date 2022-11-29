#!/usr/bin/env python3

# Copyright 2022 Shadow Robot Company Ltd.
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

import rospy
import rostopic
import rospkg
import yaml
from sr_hand_health_report.sr_hand_health_report_check import SrHealthReportCheck
from diagnostic_msgs.msg import DiagnosticArray
import time
import numpy as np
import math
from urdf_parser_py.urdf import URDF


class BacklashCheck(SrHealthReportCheck):

    THRESHOLD = 0.01
    FINGER_PWM = -250
    WIGGLING_TIME = 2

    def __init__(self, hand_side, fingers_to_test):
        super().__init__(hand_side, fingers_to_test)
        self._side_sign_map = {"ff": -1, "mf": -1, "rf": 1, "lf": 1, "th": 1}
        self.joint_limits = {}
        self._set_joint_limits()

    def _set_joint_limits(self):
        for joint in URDF.from_parameter_server().joints:
            if joint.name.lower() in [joint_name.lower() for joint_name in self._joint_msg.name]:
                self.joint_limits[joint.name.lower()] = joint.limit

    def is_wrist_included(self):
        return "WR" in self._fingers_to_joint_map.keys()

    def move_fingers_to_start_position(self):
        finger_objects = self._init_finger_objects(["FF", "MF", "RF", "TH"])
        self.switch_controller_mode('position')

        for finger in finger_objects:
            if finger.finger_name.lower() != "wr":
                finger.move_finger(0, 'position')
            if finger.finger_name.lower() in ['ff', 'mf', 'rf', 'lf']:
                for joint_index, joint_object in finger.joints_dict.items():
                    if joint_index.lower() == "j3":
                        joint_object.move_joint(math.radians(0), 'position')
                    else:
                        joint_object.move_joint(math.radians(0), 'position')

    def run_check(self):
        self.move_fingers_to_start_position()
        result = {"backlash_check": {}}

        self.switch_controller_mode("position")
        for finger_object in self.fingers_to_check:
            self.move_finger_to_side(finger_object, 'right')

        for finger_object in self.fingers_to_check:
            finger_object.move_finger(0, 'position')
            self.switch_controller_mode("effort")

            for joint in finger_object.joints_dict.values():
                rospy.logwarn(f"Running check for {joint.joint_name}")
                if finger_object.finger_name in ('ff', 'mf', 'rf', 'lf') and joint.joint_index != "j1":
                    rospy.logerr(f"Wiggling {finger_object.finger_name} {joint.joint_index}")
                    joint_result = self.wiggle_joint(joint)
                    result['backlash_check'][joint.joint_name] = joint_result
                elif finger_object.finger_name == 'th':
                    joint_result = self.wiggle_joint(joint)
                    rospy.logerr(f"Wiggling {finger_object.finger_name} {joint.joint_index}")
                    result['backlash_check'][joint.joint_name] = joint_result

            self.switch_controller_mode("position")
            self.move_finger_to_side(finger_object, 'left')

        self._result = result
        rospy.logerr(result)

    def move_finger_to_side(self, finger_object, side):
        angle = math.radians(-20) if side == 'right' else math.radians(20)
        angle *= self._side_sign_map[finger_object.finger_name]
        finger_object.joints_dict['J4'].move_joint(angle, 'position')

    def wiggle_joint(self, joint):
        test_times = []
        total_time = None
        test_count = 10  # number of iterations

        test_start_time = rospy.get_rostime()
        timeout_condition = True

        while len(test_times) < test_count and timeout_condition and not rospy.is_shutdown():

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
                    rospy.logwarn(f"{joint.get_current_position()}  {starting_position}")
                    rospy.logwarn("x1")
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
                    rospy.logwarn(f"{joint.get_current_position()}  {starting_position}")
                    rospy.logwarn("x2")
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
        result['std'] = float(np.std(times))
        result['avg'] = float(np.mean(times))
        self._result = result
        return result

    def _debounce(self, joint, time=1):
        if joint.get_current_position() < self.joint_limits[joint.joint_name].lower + math.radians(1):
            joint.move_joint(-self.FINGER_PWM, 'effort')
            rospy.sleep(time)
            rospy.logerr(f"debouncing up {joint.joint_name}")
            joint.move_joint(0, 'effort')
        if joint.get_current_position() > self.joint_limits[joint.joint_name].upper - math.radians(1):
            joint.move_joint(self.FINGER_PWM, 'effort')
            rospy.sleep(time)
            rospy.logerr(f"debouncing down {joint.joint_name}")
            joint.move_joint(0, 'effort')

    def get_result(self):
        return self._result

    def has_passed(self):
        # to be edited after value confirmation with Luke and production
        pass_value_std = 0.1
        pass_value_avg = 0.1
        passed = True

        for joint_name in self._result['backlash_check'].keys():
            joint_data = self._result['backlash_check'][joint_name]
            if joint_data['std'] < pass_value_std or joint_data['avg'] < pass_value_avg:
                passed = False
                break
        return passed


if __name__ == '__main__':
    rospy.init_node("xxxxx")

    tc = BacklashCheck('right', ["TH", "FF", "MF", "RF"])
    tc.run_check()

    for finger_object in tc.fingers_to_check:
        finger_object.move_finger(0, 'position')
