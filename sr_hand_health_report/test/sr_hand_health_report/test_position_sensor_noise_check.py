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
from sr_hand_health_report.position_sensor_noise_check import PositionSensorNoiseCheck

PKG = "sr_hand_health_report"


class TestPositionSensorNoiseCheck(TestCase):

    @classmethod
    def setUpClass(cls):
        cls.position_sensor_noise_check = PositionSensorNoiseCheck("right", "FF")

    def test_has_single_passed(self):
        test_name = next(iter(PositionSensorNoiseCheck.PASSED_THRESHOLDS))
        test_value = PositionSensorNoiseCheck.PASSED_THRESHOLDS[test_name]
        self.assertTrue(isinstance(self.position_sensor_noise_check.has_single_passed(test_name, test_value), bool))


if __name__ == "__main__":
    rospy.init_node('test_sr_hand_health_report_node', anonymous=True)
    rostest.rosrun(PKG, 'test_sr_hand_health_report', TestPositionSensorNoiseCheck)
