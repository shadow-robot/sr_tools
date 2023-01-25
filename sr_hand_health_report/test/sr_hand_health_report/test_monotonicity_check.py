#!/usr/bin/env python3

# Copyright 2023 Shadow Robot Company Ltd.
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

from unittest import TestCase
import rospy
import rostest
import numpy as np
from sr_hand_health_report.monotonicity_check import MonotonicityCheck, SW_LIMITS_FOR_JOINTS
from sr_hand_health_report.sr_hand_health_report_check import (SrHealthReportCheck,
                                                               Finger,
                                                               Joint,
                                                               SENSOR_CUTOUT_THRESHOLD,
                                                               NR_OF_BITS_NOISE_WARNING)


PKG = "sr_hand_health_report"


class TestMonotonicityCheck(TestCase):

    @classmethod
    def setUpClass(cls):
        cls.monotonicity_check = MonotonicityCheck("right", "FF")

    def test_add_result_to_dict(self):

        test_joint_name = "test_joint"
        test_joint_higher_value = 2
        test_joint_lower_value = 0.2
        self.monotonicity_check._add_result_to_dict(test_joint_name, test_joint_higher_value, test_joint_lower_value)

        result_joint_dict = self.monotonicity_check._dict_of_monotonic_joints[test_joint_name]
        self.assertTrue(result_joint_dict["is_monotonic"])
        self.assertTrue(result_joint_dict["higher_raw_sensor_value"] == test_joint_higher_value)
        self.assertTrue(result_joint_dict["lower_raw_sensor_value"] == test_joint_lower_value)

    def test_has_single_passed(self):
        test_name = next(iter(MonotonicityCheck.PASSED_THRESHOLDS))
        test_value = MonotonicityCheck.PASSED_THRESHOLDS[test_name]
        self.assertTrue(isinstance(self.monotonicity_check.has_single_passed(test_name, test_value), bool))

    def test_check_sensor_range(self):
        test_value_1 = 2
        test_value_2 = 3
        result = self.monotonicity_check.check_sensor_range(test_value_1, test_value_2)
        self.assertTrue(result[0]>=result[1])

    def test_get_raw_sensor_value(self):
        test_input = [2,3]
        expected_result = 2.5
        self.assertTrue(self.monotonicity_check.get_raw_sensor_value(test_input) == expected_result)


if __name__ == "__main__":
    rospy.init_node('test_sr_hand_health_report_node', anonymous=True)
    rostest.rosrun(PKG, 'test_sr_hand_health_report', TestMonotonicityCheck)