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
from sr_robot_msgs.msg import EthercatDebug


class OverrunCheck(SrHealthReportCheck):

    CHECK_TIME = 10  # enough time to give reasonable result

    def __init__(self, hand_side, fingers_to_test):
        super().__init__(hand_side, fingers_to_test)

        self.number_of_drops = 0
        self.iterations = 0
        self.overrun_average = 0
        self.drop_average = 0
        self._result = None

        rospy.Subscriber("/diagnostics_agg", DiagnosticArray, self.overruns_callback)
        rospy.Subscriber(f"/{self._hand_prefix}/debug_etherCAT_data", EthercatDebug, self.drops_callback)

    def overruns_callback(self, data):
        overrun = OverrunCheck.get_recent_overruns(data)

        self.overrun_average += int(float(overrun))
        self.drop_average += self.number_of_drops
        self.number_of_drops = 0
        self.iterations += 1

    def drops_callback(self, data):
        if data.sensors[10] == 0:
            self.number_of_drops += 1

    @staticmethod
    def get_recent_overruns(msg):
        for status in msg.status:
            for value_dict in status.values:
                if value_dict.key == 'Recent Control Loop Overruns':
                    return value_dict.value
        raise ValueError("\'Recent Control Loop overruns\' not present in the topic!")

    def run_check(self):
        self.overrun_average = 0
        self.drop_average = 0
        """
        start_time = rospy.get_rostime().secs
        while (rospy.get_rostime().secs - start_time) < self.CHECK_TIME:
            rospy.sleep(0.1)
        """
        rospy.sleep(self.CHECK_TIME)

        result = {}
        result["overrun_check"] = {'overrun_average': self.overrun_average, 'drop_average': self.drop_average}
        self._result = result

    def get_result(self):
        return self._result

    def has_passed(self):
        raise NotImplementedError("The function 'has_passed' must be implemented")


if __name__ == '__main__':
    rospy.init_node("sr_overrun_check")

    overrun_check = OverrunCheck('right', ['FF', 'MF', 'RF'])
    overrun_check.run_check()
    rospy.loginfo(overrun_check.get_result())
