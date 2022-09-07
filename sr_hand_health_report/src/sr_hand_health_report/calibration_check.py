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
from builtins import round
import sys
import yaml
import rospy
import rospkg
from sr_hand_health_report_check import SrHealthReportCheck
from sr_hand_health_report.position_sensor_noise_check import PositionSensorNoiseCheck
from diagnostic_msgs.msg import DiagnosticArray
import numpy as np

ZERO_JOINT_ANGLE_THRESHOLD = 0.07  # in radian


class CalibrationCheck(SrHealthReportCheck):
    def __init__(self, hand_side, fingers_to_test):
        #super().__init__(hand_side, fingers_to_test)

        try:
            self._hand_serial = rospy.get_param("~hand_serial", 2346)
        except Exception:
            rospy.logerr("No hand connected!")
            sys.exit(1)

        self.calibration = self.load_calibration()
        rospy.logwarn(self.calibration)
        self.set_fingers_to_zero()

    def load_calibration(self):
        calibration_dict = {}
        filepath = f"{rospkg.RosPack().get_path('sr_hand_config')}/{self._hand_serial}/calibrations/calibration.yaml"
        with open(filepath, 'r') as calibration_file:
            calibration = yaml.safe_load(calibration_file)
            calibration = calibration['sr_calibrations']
        for entry in range(len(calibration)):
            joint = calibration[entry][0]
            calibration_dict[joint] = {}
            calibration_values_per_joint = calibration[entry][1]
            for calibration_point in calibration_values_per_joint:
                calibration_angle = calibration_point[1]
                calibration_raw_value = calibration_point[0]
                calibration_dict[joint][str(calibration_angle)] = calibration_raw_value

        return calibration_dict

    def set_fingers_to_zero(self):
        success = True
        for finger in self.fingers_to_check:
            for joint in finger.joints_dict.values():
                self.drive_joint_to_position(joint, 0)
            rospy.sleep(2)
            for joint in finger.joints_dict.values():
                if not -ZERO_JOINT_ANGLE_THRESHOLD < joint.get_current_position() < ZERO_JOINT_ANGLE_THRESHOLD:
                    success = False
                    break
        return success

    def run_check(self):
        rospy.logwarn(self.fingers_to_check)
        for finger in self.fingers_to_check:
            self._run_check_per_finger(finger)
        return 

    def _run_check_per_finger(self, finger):
        for joint in finger.joints_dict.values():
            if finger.finger_name == "wr":
                command = 350
            else:
                command = 250
            
            joint_name = finger.finger_name + joint.joint_index
            extend_command = self.command_sign_map[joint_name]*command
            flex_command = -extend_command

            time = 5  # 5 seconds
            rate = rospy.Rate(50)  # 50 Herz
            joint_position = {}
            self.drive_joint_with_pwm(joint, extend_command, time, rate)
            joint_position['extend'] = joint.get_current_position()
            self.drive_joint_with_pwm(joint, flex_command, time, rate)
            joint_position['flex'] = joint.get_current_position()


if __name__ == "__main__":
    rospy.init_node('calibration_check_node')
    calibration_check = CalibrationCheck("right", ["FF"])
    calibration_check.run_check()
