#!/usr/bin/env python

# Copyright 2020 Shadow Robot Company Ltd.
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


import argparse
import rospy
import rospkg
import time
import yaml
import os
from sr_hand_health_report_check import SrHealthReportCheck
from monotonicity_check import MonotonicityCheck
from position_sensor_noise_check import PositionSensorNoiseCheck
from sr_hand_health_report.msg import CheckStatus
from sr_system_info.system_info import SystemInfo
from collections import OrderedDict
from rosbag_manager import RosbagManager


class HealthReportScriptNode(object):
    def __init__(self, real_hand, home_folder_path):
        self._real_hand = real_hand
        self._health_report_checks_status_publisher = rospy.Publisher("/health_report_checks_status_publisher",
                                                                      CheckStatus, queue_size=1)
        self._results = {"hand_info": {}, "checks": []}
        self._system_info = SystemInfo()
        self._system_info.collect()
        self._results["system_info"] = self._system_info.values
        self._rospack = rospkg.RosPack()
        if self._real_hand:
            self._hand_serial = self._get_hand_params()
        else:
            self._hand_serial = rospy.get_param("~hand_serial")
            self._results["hand_info"]["hand_serial"] = self._hand_serial
            self._results["hand_info"]["real_hand"] = self._real_hand
        self._check_dir_path = self._create_checks_directory(home_folder_path)
        self._results_path = "{}/{}.yml".format(
            self._check_dir_path,
            time.strftime("health_report_file"))
        self._checks_list = rospy.get_param("~checks_to_run")
        self.bag_logging_obj = RosbagManager(self._check_dir_path)
        self.bag_logging_obj.start_log()
        self._fingers_to_test = rospy.get_param("~fingers_to_test")

    def _create_checks_directory(self, home_folder_path):
        check_directory_path = "{}/sr_hand_health_reports/{}/{}".format(
                                home_folder_path,
                                self._hand_serial,
                                time.strftime("health_report_results_%Y-%m-%d_%H-%M-%S")
                                )
        if not os.path.exists(check_directory_path):
            os.makedirs(check_directory_path)
        return check_directory_path

    def _get_hand_params(self):
        hand_params = rospy.get_param("hand")
        data = hand_params.get("mapping", "")
        hand_serial = data.keys()[0]
        hand_prefix = data.values()[0]
        self._results["hand_info"]["hand_serial"] = hand_serial
        self._results["hand_info"]["hand_prefix"] = hand_prefix
        return hand_serial

    def _publish_check_status(self, check_name):
        check_status = CheckStatus()
        check_status.header.stamp = rospy.Time.now()
        check_status.check_name = check_name
        self._health_report_checks_status_publisher.publish(check_status)

    def run_monotonicity_check(self, publish_state=True):
        check_name = "monotonicity_check"
        if publish_state:
            self._publish_check_status(check_name)
        monotonicity_check = MonotonicityCheck(args.hand_side, self._fingers_to_test)
        monotonic_test_results = monotonicity_check.run_check()
        return monotonic_test_results

    def run_position_sensor_noise_check(self, publish_state=True):
        check_name = "position_sensor_noise_check"
        if publish_state:
            self._publish_check_status(check_name)
        position_sensor_noise_check = PositionSensorNoiseCheck(args.hand_side, self._fingers_to_test)
        position_sensor_noise_results = position_sensor_noise_check.run_check()
        return position_sensor_noise_results

    def write_results_to_file(self, filename=None):
        if filename is None:
            filename = self._results_path
        with open(filename, 'w') as yaml_file:
            yaml.dump(self._results, stream=yaml_file, default_flow_style=False)
        rospy.signal_shutdown("All checks completed!")

    def run_checks_real_hand(self):
        """ run all the necessary checks """
        self.bag_logging_obj.record_bag(self._check_dir_path, "health_report_bag_file")
        for check in self._checks_list:
            if check == "monotonicity_check":
                monotonic_test_results = self.run_monotonicity_check()
                self._results["checks"].append(monotonic_test_results)
            if check == "position_sensor_noise_check":
                position_sensor_noise_results = self.run_position_sensor_noise_check()
                self._results["checks"].append(position_sensor_noise_results)
            else:
                rospy.logwarn("{} not FOUND".format(check))
        self.write_results_to_file()
        self.bag_logging_obj.stop_bag()

    def run_checks_bag_file(self, rosbag_path, rosbag_name):
        self.bag_logging_obj.play_bag(rosbag_path, rosbag_name)
        rospy.wait_for_message("/health_report_checks_status_publisher", CheckStatus)
        for check in self._checks_list:
            if check == "monotonicity_check":
                monotonic_test_results = self.run_monotonicity_check(publish_state=False)
                self._results["checks"].append(monotonic_test_results)
            if check == "position_sensor_noise_check":
                position_sensor_noise_results = self.run_position_sensor_noise_check(publish_state=False)
                self._results["checks"].append(position_sensor_noise_results)
            else:
                rospy.logwarn("{} not FOUND".format(check))
        self.write_results_to_file()
        self.bag_logging_obj.stop_bag()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Run a checks for health report.')
    parser.add_argument('-hs', '--hand_side', default="right", type=str, dest='hand_side',
                        help='For which hand the checks have to be executed')
    args, unknown_args = parser.parse_known_args()
    rospy.init_node('sr_hand_health_report_script')

    real_hand = rospy.get_param("~real_hand")
    home_folder_path = rospy.get_param("~results_path")
    sr_hand_health_report_script = HealthReportScriptNode(real_hand, home_folder_path)

    if real_hand is True:
        sr_hand_health_report_script.run_checks_real_hand()
    else:
        rosbag_path = rospy.get_param("~rosbag_path")
        rosbag_name = rospy.get_param("~rosbag_name")
        sr_hand_health_report_script.run_checks_bag_file(rosbag_path, rosbag_name)
    rospy.spin()
