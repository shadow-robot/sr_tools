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

class Joint(object): 
    def __init__(self, hand_prefix, finger_name, joint_index):
        self._finger_name = finger_name
        self.joint_index = joint_index
        self._hand_prefix = hand_prefix
        self.joint_name = self._hand_prefix + "_" + self._finger_name + self.joint_index
        self.joint_index_controller = ""

        # deal with different convention sensor/controllers due to coupled joints
        if self._finger_name.upper() not in FINGERS_WITHOUT_COUPLED_JOINTS:
            if self.joint_index.upper() in COUPLED_JOINTS:
                self.joint_index_controller = "j0"
            else:
                self.joint_index_controller = self.joint_index

        self.joint_name_controller = self._hand_prefix + "_" + self._finger_name + self.joint_index_controller

        self._pwm_command_publisher = rospy.Publisher("/sh_%s_effort_controller/command" %
                                                     (self.joint_name_controller), Float64, queue_size=2)
        
        self._position_command_publisher = rospy.Publisher("/sh_%s_position_controller/command" %
                                                          (self.joint_name_controller), Float64, queue_size=2)
        self._raw_sensor_data = []
        self._current_position = float()

    def move_joint(self, command, control_type):
        if control_type is "effort":
            self._pwm_command_publisher.publish(command)
        elif control_type is "position":
            self._position_command_publisher.publish(command)

class SrHealthReportCheck(object):
    def __init__(self, hand_side):
        self._hand_prefix = hand_side[0] + "h"
        self._hand_name = hand_side + "_hand"
        self._hand_commander = SrHandCommander(name=self._hand_name)
        self._joint_states_dict = {}

        self._joint_msg = rospy.wait_for_message("/joint_states", JointState)
        self._fingers_to_joint_dict = self._init_dict_finger_joints()
        self._controller_joints_names = self._init_controller_joints()

        self.ctrl_helper = ControllerHelper([self._hand_prefix], [self._hand_prefix + "_"], self._controller_joints_names)
        self.fingers_to_check = self._init_finger_objects()

        self._raw_sensor_data_dict = {}
        self._raw_sensor_names_list = self._init_raw_sensor_data_list()

        self._raw_data_sensor_subscriber = rospy.Subscriber("/%s/debug_etherCAT_data" % (self._hand_prefix),
                                                            EthercatDebug, self._raw_data_sensor_callback)
        
        self._joint_states_subscriber = rospy.Subscriber("/joint_states", JointState,
                                                         self._joint_states_callback)

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
            fingers_to_check.append(Finger(self._hand_prefix, finger))
            for joint_index in joints:
                fingers_to_check[i].joints_dict[joint_index] = Joint(self._hand_prefix, finger.lower(),
                                                                     joint_index.lower())
        return fingers_to_check

    def _init_raw_sensor_data_list(self):
        raw_sensor_names_list = []
        for joint_name in self._joint_msg.name:
            if joint_name == (self._hand_prefix + "_THJ5"):
                for s in range(0, 2):
                    name = joint_name + "_{}".format(s)
                    raw_sensor_names_list.append(name.lower())
            elif joint_name == (self._hand_prefix + "_WRJ1"):
                for s in range(0, 2):
                    name = joint_name + "_{}".format(s)
                    raw_sensor_names_list.append(name.lower())
            else:
                raw_sensor_names_list.append(joint_name.lower())
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

    def reset_robot_to_home_position(self):
        rospy.loginfo("Sending robot to home position (open)")
        self.switch_controller_mode("trajectory")
        try:
            self._hand_commander._move_to_named_target('open', wait = True)
        except:
            rospy.logerr("Could not plan to open position")

    def drive_joint_to_position(self, joint, command):
        self.switch_controller_mode("position")
        joint.move_joint(command, "position")
        self.switch_controller_mode("effort")
        self._count_time = 0
