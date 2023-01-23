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

import time
import math
import rospy
from sr_hand_health_report.test_backlash_check import BacklashCheck
import numpy as np
from urdf_parser_py.urdf import URDF
from sensor_msgs.msg import JointState
from unittest import TestCase

class TestBacklashCheck(TestCase):

    @classmethod
    def setUpClass(cls):
        cls.backlash_check = BacklashCheck("right", "FF")

    """
        Sets the joint limits from the URDF
    """
    def test_set_joint_limits(self):
        self._set_joint_limits()
        self.assertTrue(self.joint_limit)
        self.assertTrue(isinstance(self.joint_limit, dict))

    """
        Moves tested fingers into 0 degree joint anglees in position control mode.
    """
    def test_move_fingers_to_start_position(self):
        difference_threshold = 0.1  #radians
        self.backlash_check.move_fingers_to_start_position()
        rospy.sleep(3)
        joint_states_msg = rospy.wait_for_message("/joint_states", JointState, timeout=2)
        for position in joint_states_msg.position:
            self.assertTrue(abs(position) < difference_threshold)

    """
        Runs the backlash check for all selected fingers and assings the result into the _result variable.
    """
    def test_run_check(self):        
        self.run_check()

    def test_move_finger_to_side(self):
        side = 'right'
        difference_tolerance = 0.1

        for finger in self.backlash_check._fingers_to_test:
            self.backlash_check.move_finger_to_side(finger, side)

        rospy.sleep(2)

        joint_states_msg = rospy.wait_for_message("/joint_states", JointState, timeout=2)
        for name, position in zip(joint_states_msg.name, joint_states_msg.position):
            if "J4" in name:
                self.assertTrue((abs(position) - 20) < difference_tolerance)

    def test_wiggle_joint(self):
        pass

    def test_has_passed(self):
        pass

    def test_has_single_passed(self):
        test_name = next(iter(BacklashCheck.PASSED_THRESHOLDS))
        test_value = BacklashCheck.PASSED_THRESHOLDS[test_name]
        self.assertTrue(isinstance(self.backlash_check.has_single_passed(test_name, test_value), bool))



if __name__ == '__main__':
    rospy.init_node("sr_backlash")

    backlash = BacklashCheck('right', BacklashCheck.FINGERS)
    backlash.run_check()

    for finger_element in backlash.fingers_to_check:
        finger_element.move_finger(0, 'position')
