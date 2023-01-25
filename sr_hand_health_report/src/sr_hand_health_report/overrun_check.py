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

import rospy
from sr_hand_health_report.sr_hand_health_report_check import SrHealthReportCheck
from diagnostic_msgs.msg import DiagnosticArray
from sr_robot_msgs.msg import EthercatDebug


class OverrunCheck(SrHealthReportCheck):

    CHECK_TIME = 10  # enough time to give reasonable result
    PASSED_THRESHOLDS = {'overrun_average': 1, 'drop_average': 1}

    def __init__(self, hand_side, fingers_to_test):
        """
            Initialize the OverrunCheck object
            @param hand_side: String indicating the side
            @param fingers_to_test: List of finger prefixes to test
        """
        super().__init__(hand_side, fingers_to_test)

        self._name = "Overrun"
        self.number_of_drops = 0
        self.iterations = 0
        self.overrun_average = 0
        self.drop_average = 0
        self._result = {'overrun': {}}

        rospy.Subscriber("/diagnostics_agg", DiagnosticArray, self._overruns_callback)
        rospy.Subscriber(f"/{self._hand_prefix}/debug_etherCAT_data", EthercatDebug, self._drops_callback)

    def _overruns_callback(self, data):
        """
            Overrun callback
            @param: DiagnosticArray data
        """
        overrun_value = OverrunCheck.get_recent_overruns(data)
        self.overrun_average += int(float(overrun_value))
        self.drop_average += self.number_of_drops
        self.number_of_drops = 0
        self.iterations += 1

    def _drops_callback(self, data):
        """
            Drops callback
            @param: EthercatDebug data
        """
        if data.sensors[10] == 0:
            self.number_of_drops += 1

    @staticmethod
    def get_recent_overruns(msg):
        """
            Get overruns from message
            @param: DiagnosticArray message
        """
        for status in msg.status:
            for value_dict in status.values:
                if value_dict.key == 'Recent Control Loop Overruns':
                    return value_dict.value
        raise ValueError("\'Recent Control Loop overruns\' not present in the topic!")

    def run_check(self):
        """
            Runs the check for CHECK_TIME time
        """
        self._result = {'overrun': {}}
        if self._stopped_execution:
            self._stopped_execution = False
            return
        rospy.loginfo("Running Overrun Check")

        self.overrun_average = 0
        self.drop_average = 0

        start_time = rospy.get_rostime()
        while rospy.get_rostime().secs - start_time.secs < self.CHECK_TIME:
            rospy.sleep(0.5)
            if self._stopped_execution:
                self._stopped_execution = False
                return

        self._result['overrun'] = {'overrun_average': self.overrun_average, 'drop_average': self.drop_average}
        self._stopped_execution = True
        return

    def has_passed(self):
        """
            Checks if the test execution result passed
            @return bool check passed
        """
        output = True
        for name in OverrunCheck.PASSED_THRESHOLDS:
            try:
                if not self.has_single_passed(name, self._result["overrun"][name]):
                    output = False
                    break
            except KeyError:
                output = False
        return output and bool(self._result["overrun"])

    def has_single_passed(self, name, value):
        """
            Checks if the single test execution result passed
            @param name: name of the test
            @param value: value to be compared with the thresholds
            @return bool check passed
        """
        return value < OverrunCheck.PASSED_THRESHOLDS[name]


if __name__ == '__main__':
    rospy.init_node("sr_overrun")

    overrun = OverrunCheck('right', ['FF', 'MF', 'RF'])
    overrun.run_check()
    rospy.loginfo(overrun.get_result())
