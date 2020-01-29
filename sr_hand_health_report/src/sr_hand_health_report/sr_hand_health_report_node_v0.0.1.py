#!/usr/bin/env python

import numpy
import rospy
from sr_hand_health_report_check import SrHealthReportCheck
from monotonicity_check import MonotonicityCheck

class HealthReportScriptNode(object):
    def __init__(self):
        #self._check_state_publisher = rospy.Publisher("/check_state_status_publisher", )
        pass
    
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
        monotonicity_check.run_check()
    
    def export_results(self):
        """ export the results to file """
        pass


if __name__ == "__main__":
    rospy.init_node('sr_hand_health_report_script')
    sr_hand_health_report_script = HealthReportScriptNode()
    sr_hand_health_report_script.run_checks()
    
    rospy.spin()
