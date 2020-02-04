#!/usr/bin/env python

# Copyright (C) 2019 Shadow Robot Company Ltd - All Rights Reserved. Proprietary and Confidential.
# Unauthorized copying of the content in this file, via any medium is strictly prohibited.

import rospy
from sr_hand_health_report_check import SrHealthReportCheck
from std_msgs.msg import Float64
import numpy as np
import decimal

SENSOR_NOISE = 3

class MonotonicityCheck(SrHealthReportCheck):
    def __init__(self, hand_side):
        super(MonotonicityCheck, self).__init__(hand_side)
        self._is_joint_monotonous = True
        self._dict_of_monotonic_joints = {}
        self._count_time = 0
        self._publishing_rate = rospy.Rate(50)
        self._older_raw_sensor_value = 0
        self._previous_difference = 0
        self._pwm_command = 250
        self._check_duration = rospy.Duration(6.0)

    def run_check(self):
        self.reset_robot_to_home_position()
        rospy.sleep(1.0)

        result = {"monotonicity_check" : []}
        rospy.loginfo("Running Monotonicity Check")
        self.switch_controller_mode("effort")

        # execute check for each finger and wrist
        for finger in self.fingers_to_check:
            if finger.finger_name == "FF":
                for joint in finger.joints_dict.values():
                    rospy.loginfo("Analyzing joint {}".format(joint.joint_name))

                    # For joint 4 we want to move J3 to 90 degrees, in order to allow the full range of J4
                    # without hitting other fingers
                    if joint.joint_index == "j4":
                        self.drive_joint_to_position(finger.joints_dict["J3"], 1.57)

                    self._is_joint_monotonous = True
                    time = rospy.Time.now() + self._check_duration
                    end_reached = False

                    while (rospy.Time.now() < time):
                        joint.move_joint(self._pwm_command, "effort")
                        is_joint_monotonous = self.check_monotonicity(joint)
                        if is_joint_monotonous == False:
                            self._is_joint_monotonous = False
                        self._publishing_rate.sleep()
                        if (round(rospy.Time.now().to_sec(),1) == round(time.to_sec(),1)) and end_reached == False:
                            time = rospy.Time.now() + self._check_duration
                            self._pwm_command = - self._pwm_command
                            end_reached = True
                    self._dict_of_monotonic_joints[joint.joint_name] = self._is_joint_monotonous
                    self.drive_joint_to_position(joint, 0.0)

                    if joint.joint_index == "j4":
                        self.drive_joint_to_position(finger.joints_dict["J3"], 0.0)
    
        result["monotonicity_check"].append(self._dict_of_monotonic_joints)
        rospy.loginfo("Monotonicity Check finished, exporting results")
        return result

    def check_monotonicity(self, joint):
        if self._older_raw_sensor_value == 0:
            self._older_raw_sensor_value = joint._raw_sensor_data
         
        difference_between_raw_data = joint._raw_sensor_data - self._older_raw_sensor_value
        self._older_raw_sensor_value = joint._raw_sensor_data
        if abs(difference_between_raw_data) <= SENSOR_NOISE:
            self._previous_difference = difference_between_raw_data
        else:
            if np.sign(difference_between_raw_data) != 0 and np.sign(self._previous_difference) != 0:
                if np.sign(difference_between_raw_data) != np.sign(self._previous_difference):
                    rospy.logwarn("Unmonotonic behaviour detected")
                    return False
            self._previous_difference = difference_between_raw_data
        return True
