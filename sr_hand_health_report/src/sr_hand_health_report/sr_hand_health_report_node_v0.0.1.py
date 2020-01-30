#!/usr/bin/env python

import rospy
import rospkg
import time
import yaml
from sr_hand_health_report_check import SrHealthReportCheck
from monotonicity_check import MonotonicityCheck


class HealthReportScriptNode(object):
    def __init__(self):
        #self._check_state_publisher = rospy.Publisher("/check_state_status_publisher", )
        self._results = {"tests": []}
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
        """ run all the necessary tests """
        monotonicity_check = MonotonicityCheck()
        monotonic_test_results = monotonicity_check.run_check()
        self._results["tests"].append(monotonic_test_results)
        self.write_results_to_file()
    
    def write_results_to_file(self, filename=None):
        if filename is None:
            filename = self._results_path
        with open(filename, 'w') as f:
            yaml.dump(self._results, f)

if __name__ == "__main__":
    rospy.init_node('sr_hand_health_report_script')
    sr_hand_health_report_script = HealthReportScriptNode()
    sr_hand_health_report_script.run_checks()
    
    rospy.spin()
