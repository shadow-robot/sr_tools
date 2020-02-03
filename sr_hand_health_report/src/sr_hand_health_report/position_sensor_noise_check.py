#!/usr/bin/env python

# Copyright (C) 2019 Shadow Robot Company Ltd - All Rights Reserved. Proprietary and Confidential.
# Unauthorized copying of the content in this file, via any medium is strictly prohibited.

import rospy
from sr_hand_health_report_check import SrHealthReportCheck
from multiprocessing import Process, Manager
import multiprocessing
import copy_reg
import types

WARNING_NOISE_VALUE = 3
ERROR_NOISE_VALUE = 4

class PositionSensorNoiseCheck(SrHealthReportCheck):
    def __init__(self, hand_side):
        super(PositionSensorNoiseCheck, self).__init__(hand_side)
        self._check_duration = rospy.Duration(7.0)
        self.manager = Manager()
        self._shared_dict = self.manager.dict()
        self._initial_joint_values = int()
        self._noise_result = True
        self._processes = []

    def run_check(self):
        self.reset_robot_to_home_position()
        rospy.sleep(1.0)
        result = {"position_sensor_noise_check" : []}
        rospy.loginfo("Running Position Sensor Noise Check")

        for finger in self.fingers_to_check:
            rospy.loginfo("collecting and analyzing data for FINGER {}".format(finger.finger_name))

            finger_process = []
            for joint in finger.joints_dict.values():
                self._initial_joint_values = joint._raw_sensor_data
                p = Process(target=self.check_joint_raw_sensor_value, args=(self._initial_joint_values, joint, self._shared_dict))
                finger_process.append(p)
                p.start()
            for proc in finger_process:
                proc.join()
                
        result["position_sensor_noise_check"].append(dict(self._shared_dict))
        return result

    def check_joint_raw_sensor_value(self, initial_value, joint, dictionary):
        time = rospy.Time.now() + self._check_duration
        status = "all is good"
        warning = False
       
        while (rospy.Time.now() < time):
            difference = joint._raw_sensor_data - initial_value
            if difference == WARNING_NOISE_VALUE:
                rospy.logwarn("Found value with 3 bits diff")
                status = "3 bits noise - WARNING"
            if difference >= ERROR_NOISE_VALUE:
                rospy.logerr("Found value with 4 bits diff")
                status = "{} bits noise - TEST FAILED".format(difference)
        print("Finished loop for {}".format(joint.joint_name))
        dictionary[joint.joint_name] = status
