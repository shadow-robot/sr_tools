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

import sys
import rospy
import rostopic
import rospkg
import yaml
from sr_hand_health_report.sr_hand_health_report_check import SrHealthReportCheck
from diagnostic_msgs.msg import DiagnosticArray


class TactileCheck(SrHealthReportCheck):

    _REASONABLE_RANGE = {"pst": [200, 1200], "bt_sp": [1800, 2400], "bt_2sp": [1800, 2400]}
    _FINGERS = ("FF", "MF", "RF", "LF", "TH")
    TIMEOUT = 2
    PASSED_THRESHOLDS = {'connected': True, 'reasonable': True}

    """
        Initialize the TactileCheck object
        @param hand_side: String indicating the side
    """
    def __init__(self, hand_side):
        super().__init__(hand_side, self._FINGERS)
        self._name = "Tactile"
        self._topic_name = f"/{self._hand_prefix}/tactile"
        self._result = {'tactile': {}}

        try:
            serials = rospy.get_param("/diagnostic_aggregator/load_diagnostic_analyzer/hand_serials_list")
            if len(serials) == 2:
                self._serial = serials[0] if self._hand_prefix == 'rh' else serials[1]
            else:
                self._serial = serials[0]
        except KeyError:
            pass

        try:
            self._serial
        except Exception:
            rospy.logerr("No hand detected!")
            sys.exit(1)

        self._expected_tactile_type = self.get_expected_tactile_type()
        self._topic_type_string = None
        self._fingers_to_test = self.get_fingertips_from_config_file()

    """
        Checks the expected type from general_info.yaml file
        @return: str expected fingertype
    """
    def get_expected_tactile_type(self):
        tactile_type_from_file = {}
        file_ = f"{rospkg.RosPack().get_path('sr_hand_config')}/{self._serial}/general_info.yaml"
        try:
            with open(file_, 'r', encoding="ASCII") as yaml_file:
                output = yaml.safe_load(yaml_file)
                tactile_type_from_file = list(output['sensors']['tip'].values())[0]
        except FileNotFoundError:
            rospy.logerr(f"General info for {self._serial} does not exists!")
        return tactile_type_from_file

    """
        Gets the fingertips defined in general_info.yaml file.
        @return: list
    """
    def get_fingertips_from_config_file(self):
        fingertips = {}
        file_ = f"{rospkg.RosPack().get_path('sr_hand_config')}/{self._serial}/general_info.yaml"
        try:
            with open(file_, 'r', encoding="ASCII") as yaml_file:
                output = yaml.safe_load(yaml_file)
                fingertips = [fingertip.upper() for fingertip in output['sensors']['tip']]
        except FileNotFoundError:
            rospy.logerr(f"General info for {self._serial} does not exists!")
        return fingertips

    """
        Runs check for tested fingers
    """
    def run_check(self):

        self._result["tactile"] = dict.fromkeys(self._fingers_to_test, '')

        if not self.check_if_tactile_type_match():
            self._result["tactile"] = "Wrong tactile definition in general_info.yaml!"
        else:
            for finger in self.get_fingertips_from_config_file():
                self._result["tactile"][finger] = {}
                self._result["tactile"][finger]["connected"] = self.is_sensor_connected(finger)
                self._result["tactile"][finger]["reasonable"] = self.is_reasonable(finger)
                if self._stopped_execution:
                    self._stopped_execution = False
                    return
        return

    """
        Checks the expected type from general_info.yaml file and topic type match
        @return: bool
    """
    def check_if_tactile_type_match(self):
        check = False
        try:
            self._topic_type_string = (rostopic.get_topic_type(self._topic_name)[0]).split('/')[-1]
        except AttributeError:
            self._topic_type_string = "ShadowPST"

        if self._topic_type_string == "ShadowPST" and self._expected_tactile_type == "pst":
            check = True
        elif self._topic_type_string == "BiotacAll":
            if self._expected_tactile_type == "bt_sp" or self._expected_tactile_type == "bt_2sp":
                check = True
        return check

    """
        Checks the tactile sensor is present on the finger
        @return: bool
    """
    def is_sensor_connected(self, finger):
        connected = False
        finger_to_index_mapping = {"FF": 1, "MF": 2, "RF": 3, "LF": 4, "TH": 5}
        expected_diagnostic_name = f"{self._hand_prefix} Tactile {finger_to_index_mapping[finger]}"

        now = rospy.get_time()
        while rospy.get_time() - now < TactileCheck.TIMEOUT:
            if self._stopped_execution:
                break
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
                connected = serial not in ("", "????")
                break
        return connected

    """
        Checks the values reported from the tactile sensor are within reasonable range
        @param: Finger object to be tested
        @return: bool
    """
    def is_reasonable(self, finger):
        reasonable = False
        try:
            topic_type = rostopic.get_topic_class(self._topic_name)[0]
            msg = rospy.wait_for_message(self._topic_name, topic_type, timeout=1)

            reasonable_min = self._REASONABLE_RANGE[self._expected_tactile_type][0]
            reasonable_max = self._REASONABLE_RANGE[self._expected_tactile_type][1]

            if self._topic_type_string == "ShadowPST":
                data = msg.pressure
                reasonable = reasonable_min < data[self._FINGERS.index(finger)] < reasonable_max

            elif self._topic_type_string == "BiotacAll":
                msg = msg.tactiles[self._FINGERS.index(finger)]

                if len(msg.electrodes) == sum(msg.electrodes):
                    data = msg.pac
                else:
                    data = msg.electrodes

                for value in data:
                    if not reasonable_min < value < reasonable_max:
                        reasonable = False
                        break
                    reasonable = True
        except Exception:
            pass
        return reasonable

    """
        Returns the result dictionary
        @return: dict check results
    """
    def get_result(self):
        return self._result

    """
        Checks if the test execution result passed
        @return bool check passed
    """
    def has_passed(self):

        passed = True
        for finger in self._fingers_to_test:
            if not isinstance(self._result["tactile"], dict):
                passed = False
                break
            for key in self._result["tactile"][finger].keys():
                if not self.has_single_passed(key, self.PASSED_THRESHOLDS[key]):
                    passed = False
                    break
        return passed and not isinstance(self._result["tactile"], str)

    """
        Checks if the single test execution result passed
        @param name: name of the test
        @param value: value to be compared with the thresholds
        @return bool check passed
    """
    def has_single_passed(self, name, value):
        return self.PASSED_THRESHOLDS[name] == value
