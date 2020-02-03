#!/usr/bin/env python

import argparse
import rospy
import rospkg
import time
import yaml
from sr_hand_health_report_check import SrHealthReportCheck
from monotonicity_check import MonotonicityCheck
from position_sensor_noise_check import PositionSensorNoiseCheck


class HealthReportScriptNode(object):
    def __init__(self):
        #self._check_state_publisher = rospy.Publisher("/check_state_status_publisher", )
        self._results = {"checks": []}
        rospack = rospkg.RosPack()
        self._results_path = "{}/sr_hand_health_reports/{}.yml".format(
            rospack.get_path('sr_hand_health_report'),
            time.strftime("%Y-%m-%d_%H-%M-%S"))

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
    

    def run_checks(self):
        """ run all the necessary checks """
        # monotonicity_check = MonotonicityCheck()
        # monotonic_test_results = monotonicity_check.run_check()
        # self._results["checks"].append(monotonic_test_results)

        position_sensor_noise_check = PositionSensorNoiseCheck(args.hand_side)
        position_sensor_noise_results = position_sensor_noise_check.run_check()
        self._results["checks"].append(position_sensor_noise_results)
        self.write_results_to_file()
    
    def write_results_to_file(self, filename=None):
        if filename is None:
            filename = self._results_path
        with open(filename, 'w') as f:
            yaml.dump(self._results, f, indent=5)

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Run a checks for health report.')
    parser.add_argument('-hs', '--hand_side', default="right", type=str, dest='hand_side',
                        help='For which hand the checks have to be executed')
    args, unknown_args = parser.parse_known_args()
    rospy.init_node('sr_hand_health_report_script')
    sr_hand_health_report_script = HealthReportScriptNode()
    sr_hand_health_report_script.run_checks()
    
    rospy.spin()
