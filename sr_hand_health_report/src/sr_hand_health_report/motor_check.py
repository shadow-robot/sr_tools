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
from sr_hand_health_report.sr_hand_health_report_check import SrHealthReportCheck
from diagnostic_msgs.msg import DiagnosticArray


class MotorCheck(SrHealthReportCheck):

    def __init__(self, hand_side, fingers_to_test):
        super().__init__(hand_side, fingers_to_test)
        self._topic_name = '/diagnostics_agg'

    """
        Runs the check for all fingers to test
        @return: dict the result
    """
    def run_check(self):
        result = {"motor_check": {}}

        try:
            received_msg = rospy.wait_for_message(self._topic_name, DiagnosticArray, 5)
        except rospy.ROSException:
            rospy.logerr(f"Did not receive any message on {self._topic_name} topic")
            self._result = result

        for _, message in enumerate(received_msg.status):
            if "SRDMotor" in message.name and self._hand_prefix in message.name and \
               "No motor associated to this joint" not in message.message:
                working_state = False
                split_motor_description_line = message.name.split("/")[-1].split(' ')
                motor_name = f"{split_motor_description_line[0]}_{split_motor_description_line[-1]}".lower()
                for item in message.values:
                    if "Temperature" in item.key:
                        working_state = True
                        break
                result['motor_check'][motor_name] = working_state

        self._result = result

    """
        Checks if the test execution result passed
        @return Bool value 
    """
    def has_passed(self):
        return all(result for result in self._result['motor_check'].values())

    """
        Checks if the single test execution result passed
        @return Bool value 
    """
    def has_single_passed(self):
        return all(result for result in self._result['motor_check'].values())

