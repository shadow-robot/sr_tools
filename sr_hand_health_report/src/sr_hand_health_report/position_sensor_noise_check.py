#!/usr/bin/env python3

# Copyright 2020-2022 Shadow Robot Company Ltd.
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
from sr_hand_health_report.sr_hand_health_report_check import (SrHealthReportCheck,
                                                               SENSOR_CUTOUT_THRESHOLD,
                                                               NR_OF_BITS_NOISE_WARNING)


class PositionSensorNoiseCheck(SrHealthReportCheck):
    def __init__(self, hand_side, fingers_to_test):
        super().__init__(hand_side, fingers_to_test)
        self._check_duration = rospy.Duration(5.0)
        self._shared_dict = {}
        self._publishing_rate = rospy.Rate(200)
        self._initial_raw_value = None

    def run_check(self):
        result = {"position_sensor_noise_check": []}
        rospy.loginfo("Running Position Sensor Noise Check")
        rospy.sleep(3.0)

        for finger in self.fingers_to_check:
            rospy.loginfo("collecting and analyzing data for FINGER {}".format(finger.finger_name))
            for joint in finger.joints_dict.values():
                rospy.loginfo("collecting and analyzing data for JOINT {}".format(joint.joint_name))
                self._initial_raw_value = joint.get_raw_sensor_data()
                self.check_joint_raw_sensor_value(self._initial_raw_value, joint, self._shared_dict)
        result["position_sensor_noise_check"].append(dict(self._shared_dict))
        rospy.loginfo("Position Sensor Noise Check finished, exporting results")
        self._result = result
        return result

    def check_joint_raw_sensor_value(self, initial_raw_value, joint, dictionary):
        status = ""
        test_failed = False
        if joint.joint_name != self._hand_prefix + "_wrj1" and joint.joint_name != self._hand_prefix + "_thj5":
            initial_raw_value = initial_raw_value[-1:]
        for index in range(len(initial_raw_value)):
            time = rospy.Time.now() + self._check_duration
            while rospy.Time.now() < time and test_failed is not True:
                difference = joint.get_raw_sensor_data()[index] - initial_raw_value[index]
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
            if joint.joint_name not in dictionary:
                dictionary[joint.joint_name] = status
            else:
                name = joint.joint_name + "_1"
                dictionary[name] = status

    def has_passed(self):
        return all(result for result in self._result.values())
