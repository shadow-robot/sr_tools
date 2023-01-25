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

from ast import Return
import rospy
from sr_hand_health_report.sr_hand_health_report_check import SrHealthReportCheck
from diagnostic_msgs.msg import DiagnosticArray


class MotorCheck(SrHealthReportCheck):

    PASSED_THRESHOLDS = True

    def __init__(self, hand_side, fingers_to_test):
        """
            Initialize the MotorCheck object
            @param hand_side: String indicating the side
            @param fingers_to_test: List of finger prefixes to test
        """
        super().__init__(hand_side, fingers_to_test)
        self._topic_name = '/diagnostics_agg'
        self._name = "Motor"
        self._result = {"motor": {}}

    def run_check(self):
        """
            Runs the check for all fingers to test
            @return: dict the result
        """
        self._result = {"motor": {}}
        if self._stopped_execution:
            self._stopped_execution = False
            return
        rospy.loginfo("Running Motor Check")
        result = dict(self._result)

        try:
            received_msg = rospy.wait_for_message(self._topic_name, DiagnosticArray, 5)
        except rospy.ROSException:
            rospy.logerr(f"Did not receive any message on {self._topic_name} topic")

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
                result['motor'][motor_name] = working_state
            if self._stopped_execution:
                self._stopped_execution = False
                return

        self._result = result
        self._stopped_execution = True
        return

    def has_passed(self):
        """
            Checks if the test execution result passed
            @return bool check passed
        """
        return all(result for result in self._result['motor'].values()) and bool(self._result["motor"])

    def has_single_passed(self, name, value):
        """
            Checks if the single test execution result passed
            @param name: name of the test
            @param value: value to be compared with the thresholds
            @return bool check passed
        """
        return value == MotorCheck.PASSED_THRESHOLDS
