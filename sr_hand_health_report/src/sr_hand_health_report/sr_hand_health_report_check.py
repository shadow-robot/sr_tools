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
from sr_robot_msgs.msg import EthercatDebug
from sr_controllers_tools.sr_controller_helper import ControllerHelper
from sr_robot_msgs.msg import ControlType
from sr_robot_msgs.srv import ChangeControlType

NUMBER_OF_IMU_FIELDS = 11

class Finger(object):
    def __init__(self, hand_prefix, finger_name):
        self._hand_prefix = hand_prefix
        self.finger_name = finger_name
        self.joints_dict = {}

    def move_finger(self, command):
        for index, joint in enumerate(self.joints):
            joint.move_joint(command[index])

class Joint(object): 
    def __init__(self, hand_prefix, finger_name, joint_index):
        self._finger_name = finger_name
        self.joint_index = joint_index
        self._hand_prefix = hand_prefix
        self._previous_current_position = float()
        self.joint_name = self._hand_prefix + "_" + self._finger_name + self.joint_index

        # deal with different convention sensor/controllers due to coupled joints
        if self._finger_name != "WR" and self._finger_name != "TH":
            if self.joint_index == "j1" or self.joint_index == "j2":
                self.joint_index_controller = "j0"
            else:
                self.joint_index_controller = self.joint_index

        self.joint_name_controller = self._hand_prefix + "_" + self._finger_name + self.joint_index_controller

        self._pwm_command_publisher = rospy.Publisher("/sh_%s_effort_controller/command" %
                                                     (self.joint_name_controller), Float64, queue_size=2)
        
        self._position_command_publisher = rospy.Publisher("/sh_%s_position_controller/command" %
                                                          (self.joint_name_controller), Float64, queue_size=2)
        self._end_stop_value = float()
        self._raw_sensor_data = int()

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
        self._raw_sensor_names_list = ['FFJ1', 'FFJ2', 'FFJ3', 'FFJ4', 'MFJ1', 'MFJ2', 'MFJ3', 'MFJ4',
                                       'RFJ1', 'RFJ2', 'RFJ3', 'RFJ4', 'LFJ1', 'LFJ2', 'LFJ3', 'LFJ4', 'LFJ5',
                                       'THJ1', 'THJ2', 'THJ3', 'THJ4', 'THJ5_1', 'THJ5_2', 'WRJ1_1', 'WRJ1_2', 'WRJ2']

        self._raw_data_sensor_subscriber = rospy.Subscriber("/%s/debug_etherCAT_data" % (self._hand_prefix),
                                                            EthercatDebug, self._raw_data_sensor_callback)

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
                if finger != "WR" and finger != "TH":
                    if joint_index == "J1" or joint_index == "J2":
                        joint_index = "J0"
                full_joint_name = finger + joint_index
                controller_joints_names.append(full_joint_name.lower())
        return controller_joints_names

    def _init_finger_objects(self):
        fingers_to_check = []
        for i, (finger, joints) in enumerate(self._fingers_to_joint_dict.items()):
            fingers_to_check.append(Finger(self._hand_prefix, finger))
            for joint_index in joints:
                fingers_to_check[i].joints_dict[joint_index] = Joint(self._hand_prefix, finger.lower(), joint_index.lower())
        return fingers_to_check

    def _raw_data_sensor_callback(self, ethercat_data):
        for i in range(0, len(ethercat_data.sensors)- NUMBER_OF_IMU_FIELDS):
            joint_name = self._hand_prefix + "_" + self._raw_sensor_names_list[i]
            self._raw_sensor_data_dict[joint_name.lower()] = ethercat_data.sensors[i]

        for finger in self.fingers_to_check:
            for joint in finger.joints_dict.values():
                if joint.joint_name == self._hand_prefix + "_thj5":
                    sensor_average = (self._raw_sensor_data_dict[self._hand_prefix + '_thj5_1'] + \
                                      self._raw_sensor_data_dict[self._hand_prefix + '_thj5_2']) / 2
                    joint._raw_sensor_data = sensor_average
                elif joint.joint_name == self._hand_prefix + "_wrj1":
                    sensor_average = (self._raw_sensor_data_dict[self._hand_prefix + '_wrj1_1'] + \
                                      self._raw_sensor_data_dict[self._hand_prefix + '_wrj1_2']) / 2
                    joint._raw_sensor_data = sensor_average
                else:
                    joint._raw_sensor_data = self._raw_sensor_data_dict[joint.joint_name]

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
        """
        After a test send the robot to home position, to execute a new test
        """
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
