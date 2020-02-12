#!/usr/bin/env python

# Copyright (C) 2019 Shadow Robot Company Ltd - All Rights Reserved. Proprietary and Confidential.
# Unauthorized copying of the content in this file, via any medium is strictly prohibited.

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
from sr_robot_msgs.msg import EthercatDebug
from sr_controllers_tools.sr_controller_helper import ControllerHelper
from sr_robot_msgs.msg import ControlType
from sr_robot_msgs.srv import ChangeControlType

COUPLED_JOINTS = ["J1", "J2"]
FINGERS_WITHOUT_COUPLED_JOINTS = ["WR", "TH"]
SENSOR_CUTOUT_THRESHOLD = 200
NR_OF_BITS_NOISE_WARNING = 3

class Finger(object):
    def __init__(self, hand_prefix, finger_name):
        self._hand_prefix = hand_prefix
        self.finger_name = finger_name
        self.joints_dict = OrderedDict()
    
    def move_finger(self, command, control_type):
        for j in self.joints_dict.values():
            j.move_joint(command, control_type)

class Joint(object):
    def __init__(self, hand_prefix, finger_name, joint_index):
        self._finger_name = finger_name
        self.joint_index = joint_index
        self._hand_prefix = hand_prefix
        self.joint_name = self._hand_prefix + "_" + self._finger_name + self.joint_index
        self.joint_index_controller = ""

        # deal with different convention sensor/controllers due to coupled joints
        self.joint_index_controller = self.joint_index
        if self._finger_name.upper() not in FINGERS_WITHOUT_COUPLED_JOINTS:
            if self.joint_index.upper() in COUPLED_JOINTS:
                self.joint_index_controller = "j0"

        self.joint_name_controller = self._hand_prefix + "_" + self._finger_name + self.joint_index_controller

        self._pwm_command_publisher = rospy.Publisher("/sh_%s_effort_controller/command" %
                                                     (self.joint_name_controller), Float64, queue_size=2)
        
        self._position_command_publisher = rospy.Publisher("/sh_%s_position_controller/command" %
                                                          (self.joint_name_controller), Float64, queue_size=2)
        self._raw_sensor_data = []
        self._current_position = float()

    def move_joint(self, command, control_type):
        print("joint name: ", self.joint_name_controller)
        print("Comamnd: ", command)
        if control_type is "effort":
            self._pwm_command_publisher.publish(command)
        elif control_type is "position":
            self._position_command_publisher.publish(command)

class SrHealthReportCheck(object):
    def __init__(self, hand_side):
        self._hand_prefix = hand_side[0] + "h"
        self._hand_name = hand_side + "_hand"
        self._joint_states_dict = {}

        self._joint_msg = rospy.wait_for_message("/joint_states", JointState)
        self._fingers_to_joint_dict = self._init_dict_finger_joints()
        self._controller_joints_names = self._init_controller_joints()

        self.ctrl_helper = ControllerHelper([self._hand_prefix], [self._hand_prefix + "_"],
                                            self._controller_joints_names)
        self.fingers_to_check = self._init_finger_objects()

        self._raw_sensor_data_dict = OrderedDict()
        self._raw_sensor_names_list = self._init_raw_sensor_data_list()

        self._raw_data_sensor_subscriber = rospy.Subscriber("/%s/debug_etherCAT_data" % (self._hand_prefix),
                                                            EthercatDebug, self._raw_data_sensor_callback)

        self._joint_states_subscriber = rospy.Subscriber("/joint_states", JointState,
                                                         self._joint_states_callback)

        if self._hand_prefix == "lh":
            self.command_sign_dict = {"ffj1": -1, "ffj2": -1, "ffj3": 1, "ffj4": -1,
                                      "mfj1": -1, "mfj2": -1, "mfj3": 1, "mfj4": -1,
                                      "rfj1": -1, "rfj2": -1, "rfj3": 1, "rfj4": -1,
                                      "lfj1": 1, "lfj2": 1, "lfj3": -1, "lfj4": 1, "lfj5": 1,
                                      "thj1": 1, "thj2": 1, "thj3": -1, "thj4": 1, "thj5": 1,
                                      "wrj1": -1, "wrj2": -1}
        elif self._hand_prefix == "rh":
            self.command_sign_dict = {"ffj1": 1, "ffj2": 1, "ffj3": -1, "ffj4": 1,
                                      "mfj1": 1, "mfj2": 1, "mfj3": -1, "mfj4": 1,
                                      "rfj1": 1, "rfj2": -1, "rfj3": -1, "rfj4": 1,
                                      "lfj1": -1, "lfj2": -1, "lfj3": 1, "lfj4": -1, "lfj5": -1,
                                      "thj1": -1, "thj2": -1, "thj3": 1, "thj4": -1, "thj5": -1,
                                      "wrj1": -1, "wrj2": 1}
    def _init_dict_finger_joints(self):
        fingers_to_joint_dict = OrderedDict()
        for joint in self._joint_msg.name:
            finger_name = joint[3:-2]
            if finger_name not in fingers_to_joint_dict:
                fingers_to_joint_dict[finger_name] = []
            fingers_to_joint_dict[finger_name].append(joint[-2:])
        return fingers_to_joint_dict

    def _init_controller_joints(self):
        controller_joints_names = []
        for finger, joint in self._fingers_to_joint_dict.items():
            for joint_index in joint:
                # deal with different convention sensor/controllers due to coupled joints
                if finger not in FINGERS_WITHOUT_COUPLED_JOINTS:
                    if joint_index in COUPLED_JOINTS:
                        joint_index = "J0"
                full_joint_name = finger + joint_index
                controller_joints_names.append(full_joint_name.lower())
        return controller_joints_names

    def _init_finger_objects(self):
        fingers_to_check = []
        for i, (finger, joints) in enumerate(self._fingers_to_joint_dict.items()):
            fingers_to_check.append(Finger(self._hand_prefix, finger.lower()))
            for joint_index in joints:
                fingers_to_check[i].joints_dict[joint_index] = Joint(self._hand_prefix, finger.lower(),
                                                                     joint_index.lower())
        return fingers_to_check

    def _init_raw_sensor_data_list(self):
        raw_sensor_names_list = []
        if "FF" in self._fingers_to_joint_dict:
            for joint_index in self._fingers_to_joint_dict["FF"]:
                name_to_append = self._hand_prefix + "_FF" + joint_index
                raw_sensor_names_list.append(name_to_append.lower())
        if "MF" in self._fingers_to_joint_dict:
            for joint_index in self._fingers_to_joint_dict["MF"]:
                name_to_append = self._hand_prefix + "_MF" + joint_index
                raw_sensor_names_list.append(name_to_append.lower())
        if "RF" in self._fingers_to_joint_dict:
            for joint_index in self._fingers_to_joint_dict["RF"]:
                name_to_append = self._hand_prefix + "_RF" + joint_index
                raw_sensor_names_list.append(name_to_append.lower())
        if "LF" in self._fingers_to_joint_dict:
            for joint_index in self._fingers_to_joint_dict["LF"]:
                name_to_append = self._hand_prefix + "_LF" + joint_index
                raw_sensor_names_list.append(name_to_append.lower())
        if "TH" in self._fingers_to_joint_dict:
            for joint_index in self._fingers_to_joint_dict["TH"]:
                if joint_index == "J5":
                    for s in range(0, 2):
                        name_to_append = self._hand_prefix + "_TH" + joint_index + "_{}".format(s)
                        raw_sensor_names_list.append(name_to_append.lower())
                else:
                    name_to_append = self._hand_prefix + "_TH" + joint_index
                    raw_sensor_names_list.append(name_to_append.lower())
        if "WR" in self._fingers_to_joint_dict:
            for joint_index in self._fingers_to_joint_dict["WR"]:
                if joint_index == "J1":
                    for s in range(0, 2):
                        name_to_append = self._hand_prefix + "_WR" + joint_index + "_{}".format(s)
                        raw_sensor_names_list.append(name_to_append.lower())
                else:
                    name_to_append = self._hand_prefix + "_WR" + joint_index
                    raw_sensor_names_list.append(name_to_append.lower())
        return raw_sensor_names_list

    def _joint_states_callback(self, sensor_msg):
        count = 0
        for finger in self.fingers_to_check:
            for joint in finger.joints_dict.values():
                if joint.joint_name == sensor_msg.name[count].lower():
                    joint._current_position = sensor_msg.position[count]
                count += 1

    def _raw_data_sensor_callback(self, ethercat_data):
        for i in range(0, len(self._raw_sensor_names_list)):
            self._raw_sensor_data_dict[self._raw_sensor_names_list[i]] = ethercat_data.sensors[i]
        
        for finger in self.fingers_to_check:
            for joint in finger.joints_dict.values():
                raw_sensor_data_list = []
                if joint.joint_name == self._hand_prefix + "_thj5":
                    raw_sensor_data_list.append(self._raw_sensor_data_dict[self._hand_prefix + '_thj5_0'])
                    raw_sensor_data_list.append(self._raw_sensor_data_dict[self._hand_prefix + '_thj5_1'])
                elif joint.joint_name == self._hand_prefix + "_wrj1":
                    raw_sensor_data_list.append(self._raw_sensor_data_dict[self._hand_prefix + '_wrj1_0'])
                    raw_sensor_data_list.append(self._raw_sensor_data_dict[self._hand_prefix + '_wrj1_1'])
                else:
                    raw_sensor_data_list.insert(0, self._raw_sensor_data_dict[joint.joint_name])
                joint._raw_sensor_data = raw_sensor_data_list


    def switch_controller_mode(self, control_type):
        if control_type is "trajectory":
            rospy.loginfo("Changing trajectory controllers to RUN")
            self.ctrl_helper.change_trajectory_ctrl("run")
        elif control_type is "position" or control_type is "effort":
            self.ctrl_helper.change_trajectory_ctrl("stop")
            change_type_msg = ChangeControlType()
            change_type_msg.control_type = ControlType.PWM
            self.ctrl_helper.change_force_ctrl_type(change_type_msg)
            rospy.loginfo("Changing controllers to: %s", control_type)
            self.ctrl_helper.change_hand_ctrl(control_type)

    def drive_joint_to_position(self, joint, command):
        self.switch_controller_mode("position")
        joint.move_joint(command, "position")
        self.switch_controller_mode("effort")
        self._count_time = 0
    
    def drive_joint_with_pwm(self, joint, command):
        print("Moving joint with pwM function: ", joint.joint_name)
        print("Command: ", command)
        joint.move_joint(command, "effort")
        self._count_time = 0
