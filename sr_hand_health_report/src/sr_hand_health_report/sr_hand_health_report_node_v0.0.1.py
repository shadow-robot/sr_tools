#!/usr/bin/env python

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

class HealthReportScriptNode(object):
    def __init__(self):
        self._health_report_checks_status_publisher = rospy.Publisher("/health_report_checks_status_publisher", CheckStatus, queue_size=1)
        self._results = {"hand_info": {}, "health_report_script_version": "v0.01", "checks": []}
        self._system_info = SystemInfo()
        self._system_info.collect()
        self._results["system_info"] = self._system_info.values
        rospack = rospkg.RosPack()
        self._hand_serial = self._get_hand_params()
        self._hand_directory_path = "{}/sr_hand_health_reports/{}".format(
                                    rospack.get_path('sr_hand_health_report'),
                                    self._hand_serial)
        if not os.path.exists(self._hand_directory_path):
            os.makedirs(self._hand_directory_path)
        self._results_path = "{}/sr_hand_health_reports/{}/{}.yml".format(
            rospack.get_path('sr_hand_health_report'),
            self._hand_serial,
            time.strftime("health_report_results_%Y-%m-%d_%H-%M-%S"))

    def _get_hand_params(self):
        hand_params = rospy.get_param("hand")
        data = hand_params.get("mapping", "")
        hand_serial =  data.keys()[0]
        hand_prefix = data.values()[0]
        self._results["hand_info"]["hand_serial"] = hand_serial
        self._results["hand_info"]["hand_prefix"] = hand_prefix
        return hand_serial

    def _read_from_real_hand(self):
        """
        Execute checks by reading data from real hand
        """
        pass

    def _read_from_bag_file(self):
        """
        Execute checks by reading data from bag_file
        """
        pass
    
    def publish_check_status(self, check_name):
        check_status = CheckStatus()
        check_status.header.stamp = rospy.Time.now()
        check_status.check_name = check_name
        self._health_report_checks_status_publisher.publish(check_status)

    def run_checks(self):
        """ run all the necessary checks """
        # check_name = "monotonicity_check"
        # self.publish_check_status(check_name)
        # monotonicity_check = MonotonicityCheck(args.hand_side)
        # monotonic_test_results = monotonicity_check.run_check()
        # self._results["checks"].append(monotonic_test_results)

        # check_name = "position_sensor_noise_check"
        # self.publish_check_status(check_name)
        # position_sensor_noise_check = PositionSensorNoiseCheck(args.hand_side)
        # position_sensor_noise_results = position_sensor_noise_check.run_check()
        # self._results["checks"].append(position_sensor_noise_results)
        self.write_results_to_file()

    def write_results_to_file(self, filename=None):
        if filename is None:
            filename = self._results_path
        with open(filename, 'w') as yaml_file:
            yaml.dump(self._results, stream=yaml_file, default_flow_style=False)
        rospy.signal_shutdown("All checks completed!"); 

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Run a checks for health report.')
    parser.add_argument('-hs', '--hand_side', default="right", type=str, dest='hand_side',
                        help='For which hand the checks have to be executed')
    args, unknown_args = parser.parse_known_args()
    rospy.init_node('sr_hand_health_report_script')
    sr_hand_health_report_script = HealthReportScriptNode()
    sr_hand_health_report_script.run_checks()
    
    rospy.spin()
