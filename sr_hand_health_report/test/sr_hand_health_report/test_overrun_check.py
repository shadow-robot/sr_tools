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

from sys import breakpointhook
import rospy
from sr_hand_health_report.sr_hand_health_report_check import SrHealthReportCheck
from diagnostic_msgs.msg import DiagnosticArray
from sr_robot_msgs.msg import EthercatDebug


class OverrunCheck(SrHealthReportCheck):

    CHECK_TIME = 10  # enough time to give reasonable result
    PASSED_THRESHOLDS = {'overrun_average': 1, 'drop_average': 1}

    def __init__(self, hand_side, fingers_to_test):
        super().__init__(hand_side, fingers_to_test)

        self.number_of_drops = 0
        self.iterations = 0
        self.overrun_average = 0
        self.drop_average = 0
        self._result = None

        rospy.Subscriber("/diagnostics_agg", DiagnosticArray, self._overruns_callback)
        rospy.Subscriber(f"/{self._hand_prefix}/debug_etherCAT_data", EthercatDebug, self._drops_callback)

    """
        Overrun callback
        @param: DiagnosticArray data
    """
    def _overruns_callback(self, data):
        overrun = OverrunCheck.get_recent_overruns(data)

        self.overrun_average += int(float(overrun))
        self.drop_average += self.number_of_drops
        self.number_of_drops = 0
        self.iterations += 1

    """
        Drops callback
        @param: EthercatDebug data
    """
    def _drops_callback(self, data):
        if data.sensors[10] == 0:
            self.number_of_drops += 1

    """
        Get overruns from message
        @param: DiagnosticArray message
    """
    @staticmethod
    def get_recent_overruns(msg):
        for status in msg.status:
            for value_dict in status.values:
                if value_dict.key == 'Recent Control Loop Overruns':
                    return value_dict.value
        raise ValueError("\'Recent Control Loop overruns\' not present in the topic!")

    """
        Runs the check for CHECK_TIME time
    """
    def run_check(self):
        self.overrun_average = 0
        self.drop_average = 0

        start_time = rospy.get_rostime()
        while rospy.get_rostime().secs - start_time.secs < self.CHECK_TIME:
            rospy.sleep(0.5)
            if self._stopped_execution:
                self._stopped_execution = False
                return {}
    
        result = {}
        result["overrun"] = {'overrun_average': self.overrun_average, 'drop_average': self.drop_average}

        self._result = result

    def has_passed(self):
        output = True
        for name in OverrunCheck.PASSED_THRESHOLDS:
            if not self.has_single_passed(name, self._result["overrun"][name]):
                output = False
                break
        return output

    def has_single_passed(self, name, value):
        return value < OverrunCheck.PASSED_THRESHOLDS[name]



if __name__ == '__main__':
    rospy.init_node("sr_overrun")

    overrun = OverrunCheck('right', ['FF', 'MF', 'RF'])
    overrun.run_check()
    rospy.loginfo(overrun.get_result())
