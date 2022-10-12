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

import sys
import rospy
import rostopic
import rospkg
import yaml
from sr_hand_health_report.sr_hand_health_report_check import SrHealthReportCheck
from diagnostic_msgs.msg import DiagnosticArray


class TactileCheck(SrHealthReportCheck):

    _REASONABLE_RANGE = {"pst": [300, 1200], "bt_sp": [1800, 2400], "bt_2sp": [1800, 2400]}
    _FINGERS = ("FF", "MF", "RF", "LF", "TH")

    def __init__(self, hand_side, fingers_to_test):
        super().__init__(hand_side, fingers_to_test)
        self._topic_name = f"/{self._hand_prefix}/tactile"
        try:
            self._serial = rospy.get_param(f"/sr_hand_robot/{self._hand_prefix}/hand_serial")
        except KeyError:
            rospy.logerr("No hand detected!")
            sys.exit(1)
        self._expected_tactile_type = self.get_expected_tactile_type()
        self._topic_type_string = None

    def get_expected_tactile_type(self):
        tactile_type_from_file = {}
        file = f"{rospkg.RosPack().get_path('sr_hand_config')}/{self._serial}/general_info.yaml"
        try:
            with open(file, 'r', encoding="ASCII") as yaml_file:
                output = yaml.safe_load(yaml_file)
                tactile_type_from_file = list(output['sensors']['tip'].values())[0]
        except FileNotFoundError:
            rospy.logerr(f"General info for {self._serial} does not exists!")
        return tactile_type_from_file

    def run_check(self):
        result = {"tactile_check": {}}
        result["tactile_check"] = dict.fromkeys(self._fingers_to_test, '')

        if not self.check_if_tactile_type_match():
            result["tactile_check"] = "Wrong tactile definition in general_info.yaml!"
        else:
            for finger in self._fingers_to_test:
                result["tactile_check"][finger] = {}
                result["tactile_check"][finger]["connected"] = self.is_sensor_connected(finger)
                result["tactile_check"][finger]["reasonable"] = self.is_reasonable(finger)
        self._result = result
        return result

    def check_if_tactile_type_match(self):
        check = False
        self._topic_type_string = (rostopic.get_topic_type(self._topic_name)[0]).split('/')[-1]

        if self._topic_type_string == "ShadowPST" and self._expected_tactile_type == "pst":
            check = True
        elif self._topic_type_string == "BiotacAll":
            if self._expected_tactile_type == "bt_sp" or self._expected_tactile_type == "bt_2sp":
                check = True
        return check

    def is_sensor_connected(self, finger):
        connected = False
        finger_to_index_mapping = {"FF": 1, "MF": 2, "RF": 3, "LF": 4, "TH": 5}
        expected_diagnostic_name = f"{self._hand_prefix} Tactile {finger_to_index_mapping[finger]}"

        now = rospy.get_time()
        timeout = 2
        while rospy.get_time() - now < timeout:
            # 2s to give more time for messages to arrive due to low publishing rate of /diagnostics
            diagnostic_data = None
            try:
                diagnostic_msg = rospy.wait_for_message('/diagnostics', DiagnosticArray, timeout=1)
                diagnostic_data = [msg for msg in diagnostic_msg.status if msg.name == expected_diagnostic_name]
            except rospy.exceptions.ROSException:
                pass
            if diagnostic_data:
                details_dict = {}
                for entry in diagnostic_data[0].values:
                    details_dict.update({entry.key: entry.value})
                serial = details_dict['Serial Number'][-4:]
                connected = not serial in ("", "????")
                break
        return connected

    def is_reasonable(self, finger):
        reasonable = True
        try:
            topic_type = rostopic.get_topic_class(self._topic_name)[0]
            msg = rospy.wait_for_message(self._topic_name, topic_type, timeout=1)

            if self._topic_type_string == "ShadowPST":
                reasonable_min = self._REASONABLE_RANGE[self._expected_tactile_type][0]
                reasonable_max = self._REASONABLE_RANGE[self._expected_tactile_type][1]
                data = msg.pressure
                reasonable = reasonable_min < data[self._FINGERS.index(finger)] < reasonable_max

            elif self._topic_type_string == "BiotacAll":
                msg = msg.tactiles[self._FINGERS.index(finger)]
                reasonable_min = self._REASONABLE_RANGE[self._expected_tactile_type][0]
                reasonable_max = self._REASONABLE_RANGE[self._expected_tactile_type][1]

                if len(msg.electrodes) == sum(msg.electrodes):
                    data = msg.pac
                else:
                    data = msg.electrodes

                for value in data:
                    if not reasonable_min < value < reasonable_max:
                        reasonable = False
        except Exception:
            pass
        return reasonable

    def get_result(self):
        return self._result

    def has_passed(self):
        passed = True
        for finger in self._fingers_to_test:
            if not isinstance(self._result["tactile_check"], dict):
                passed = False
                break
            for key in self._result["tactile_check"][finger].keys():
                if not self._result["tactile_check"][finger][key]:
                    passed = False
                    break
        return passed
