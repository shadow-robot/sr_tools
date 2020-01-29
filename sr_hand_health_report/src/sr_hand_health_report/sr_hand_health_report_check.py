#!/usr/bin/env python

import rospy
import os
from controller_manager_msgs.srv import ListControllers
from controller_manager_msgs.srv import SwitchController, LoadController, UnloadController
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from sr_robot_commander.sr_hand_commander import SrHandCommander
from std_srvs.srv import SetBool
from collections import OrderedDict 
from multiprocessing import Process


class Finger(object):
    def __init__(self, hand_prefix, finger_name, joint_end_stop_values):
        self._hand_prefix = hand_prefix
        self.finger_name = finger_name
        self.joints = []
        self.joint_end_stop_values = joint_end_stop_values

    def move_finger(self, command):
        for index, joint in enumerate(self.joints):
            joint.move_joint(command[index])
    
    def populate_joint_end_stop_values(self):
        for joint_object in self.joints:
            for joint, value in self.joint_end_stop_values.items():
                if joint_object.joint_index == joint:
                    joint_object._end_stop_value = value


class Joint(object): 
    def __init__(self, hand_prefix, finger_name, joint_index):
        self._finger_name = finger_name
        self.joint_index = joint_index
        self._hand_prefix = hand_prefix
        
        self.joint_name = self._hand_prefix + "_" + self._finger_name + self.joint_index

        # deal with different convention sensor/controllers due to coupled joints
        if self.joint_index == "j1" or self.joint_index == "j2":
            self.joint_index_controller = "j0"
        else:
            self.joint_index_controller = self.joint_index
        
        self.joint_name_controller = self._hand_prefix + "_" + self._finger_name + self.joint_index_controller

        self._pwm_command_publisher = rospy.Publisher("/sh_%s_effort_controller/command" %
                                                     (self.joint_name_controller), Float64, queue_size=2)
        self._end_stop_value = float()
        self._raw_sensor_data = float()
        self._joint_states_dict = {}
        self.end_stop_reached_flag = False

    def move_joint(self, command):

        self._pwm_command_publisher.publish(command)

    def end_stop_reached(self, current_position):
        """
        check whether an end stop has been reached
        """
        if abs(current_position) - abs(self._end_stop_value) < 0.01:  #make threshold variable
            self.end_stop_reached_flag = True


class SrHealthReportCheck(object):
    def __init__(self, hand_prefix):
        self._hand_prefix = hand_prefix
        self._hand_commander = SrHandCommander(name="left_hand")
        self._joint_states_dict = {}
        self._fingers_to_joint_dict = OrderedDict([
                                                   ("FF", ['J1', 'J2', 'J3', 'J4']),
                                                   ("MF", ['J1', 'J2', 'J3', 'J4']),
                                                   ("RF", ['J1', 'J2', 'J3', 'J4']),
                                                   ("LF", ['J1', 'J2', 'J3', 'J4', 'J5']),
                                                   ("TH", ['J1', 'J2', 'J3', 'J4', 'J5']),
                                                   ("WR", ['J1', 'J2'])
                                                  ])
        self._fingers_endstop_limits = {'J1': 1.4, 'J2': 1.58, 'J3': 1.57, 'J4': 0.0, 'J5': 0.79}  #0.34
        self._thumb_endstop_limits = {'J1': 1.52, 'J2': 0.72, 'J3': 0.20, 'J4': 1.11, 'J5':-0.925}
        self._wrist_endstop_limits = {'J1': 0.5, 'J2': 0.17}

        self.fingers_to_check = self._init_finger_objects()

        self._raw_data_sensor_subscriber = rospy.Subscriber("/%s/debug_etherCAT_data/sensors" % (self._hand_prefix),
                                                            Float64, self._raw_data_sensor_callback)
        self._joint_state_subscriber = rospy.Subscriber("/joint_states", JointState, self._joint_state_callback)

    def _init_finger_objects(self):
        fingers_to_check = []
        for i, (finger, joints) in enumerate(self._fingers_to_joint_dict.items()):
            if finger == "TH":
                fingers_to_check.append(Finger(self._hand_prefix, finger, self._thumb_endstop_limits))
            elif finger == "WR":
                fingers_to_check.append(Finger(self._hand_prefix, finger, self._wrist_endstop_limits))
            else:
                fingers_to_check.append(Finger(self._hand_prefix, finger, self._fingers_endstop_limits))
            for joint_index in joints:
                fingers_to_check[i].joints.append(Joint(self._hand_prefix, finger.lower(), joint_index.lower()))
            fingers_to_check[i].populate_joint_end_stop_values()
        return fingers_to_check

    def _raw_data_sensor_callback(self, raw_sensor_array):
        for i, joint in enumerate(self.joints_to_check):
            joint._raw_sensor_data = raw_sensor_array[i]

    def _joint_state_callback(self, joint_sensor_msg):
        for i, name in enumerate(joint_sensor_msg.name):
            self._joint_states_dict[name.lower()] = joint_sensor_msg.position[i]

        for finger in self.fingers_to_check:
            for joint in finger.joints:
                joint.end_stop_reached(self._joint_states_dict[joint.joint_name])

    def reset_robot_to_home_position(self):
        """
        After a test send the robot to home position, to execute a new test
        """
        self._hand_commander.move_to_named_target('open')

    def run_test(self):
        """
        execute the test
        """
        pass

    def export_result(self):
        """
        save the test result to file
        """
        pass
