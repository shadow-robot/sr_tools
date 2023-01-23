#!/usr/bin/env python3

# Copyright 2020-2023 Shadow Robot Company Ltd.
#
# This program is free software: you can redistribute it and/or modify it
# under the terms of the GNU General Public License as published by the Free
# Software Foundation version 2 of the License.
#
# This program is distributed in the hope that it will be useful, but WITHOUT
# ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
# FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
# more details.
#
# You should have received a copy of the GNU General Public License along
# with this program. If not, see <http://www.gnu.org/licenses/>.
from collections import OrderedDict
from operator import indexOf
from std_msgs.msg import Float64
import rospy
from sensor_msgs.msg import JointState
from sr_controllers_tools.sr_controller_helper import ControllerHelper
from sr_robot_msgs.msg import EthercatDebug
from sr_robot_msgs.msg import ControlType
from sr_robot_msgs.srv import ChangeControlType
import math

COUPLED_JOINTS = ["J1", "J2"]
FINGERS_WITHOUT_COUPLED_JOINTS = ["WR", "TH"]
SENSOR_CUTOUT_THRESHOLD = 200
NR_OF_BITS_NOISE_WARNING = 3


class Finger:

    PREFIXES = ("FF", 'MF', 'RF', 'LF', 'TH', 'WR')

    def __init__(self, hand_prefix, finger_name):
        self._hand_prefix = hand_prefix
        self.finger_name = finger_name
        self.joints_dict = OrderedDict()

    def move_finger(self, command, control_type):
        for joint in self.joints_dict.values():
            joint.move_joint(command, control_type)

    def _get_sorting_value(self):
        return self.PREFIXES.index(self.finger_name.upper())


class Joint:

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

        self._pwm_command_publisher = rospy.Publisher(f"/sh_{self.joint_name_controller}_effort_controller/command",
                                                      Float64, queue_size=2)

        self._position_command_publisher = rospy.Publisher(f"/sh_{self.joint_name_controller}_"
                                                           f"position_controller/command", Float64, queue_size=2)
        self._raw_sensor_data = []
        self._current_position = float()

    def move_joint(self, command, control_type):
        if control_type == "effort":
            self._pwm_command_publisher.publish(command)
        elif control_type == "position":
            self._position_command_publisher.publish(command)

    def set_current_position(self, new_position):
        self._current_position = new_position

    def get_current_position(self):
        return self._current_position

    def set_raw_sensor_data(self, new_raw_sensor_data):
        self._raw_sensor_data = new_raw_sensor_data

    def get_raw_sensor_data(self):
        return self._raw_sensor_data


class SrHealthReportCheck:
    def __init__(self, hand_side, fingers_to_test):
        self._hand_prefix = hand_side[0] + "h"
        self._hand_name = hand_side + "_hand"
        self._name = ""
        self._result = None
        self._stopped_execution = False

        self._joint_msg = rospy.wait_for_message("/joint_states", JointState)
        self._fingers_to_joint_map = self._init_map_finger_joints()

        self._controller_joints_names = self._init_controller_joints()

        self.ctrl_helper = ControllerHelper([self._hand_prefix], [self._hand_prefix + "_"],
                                            self._controller_joints_names)
        self.ctrl_helper.time_to_reload_params = 1.0
        self._fingers_to_test = fingers_to_test
        self.fingers_to_check = self._init_finger_objects()

        self._raw_sensor_data_map = OrderedDict()
        self._raw_sensor_names_list = self._init_raw_sensor_data_list()

        self._raw_data_sensor_subscriber = rospy.Subscriber("/%s/debug_etherCAT_data" % (self._hand_prefix),
                                                            EthercatDebug, self._raw_data_sensor_callback)

        self._joint_states_subscriber = rospy.Subscriber("/joint_states", JointState,
                                                         self._joint_states_callback)

        self._side_sign_map = {"ff": -1, "mf": -1, "rf": 1, "lf": 1, "th": 1, "wr": 1}

        if self._hand_prefix == "lh":
            self.command_sign_map = {"ffj1": -1, "ffj2": -1, "ffj3": 1, "ffj4": -1,
                                     "mfj1": -1, "mfj2": -1, "mfj3": 1, "mfj4": -1,
                                     "rfj1": -1, "rfj2": -1, "rfj3": 1, "rfj4": -1,
                                     "lfj1": 1, "lfj2": 1, "lfj3": -1, "lfj4": 1, "lfj5": 1,
                                     "thj1": 1, "thj2": 1, "thj3": -1, "thj4": 1, "thj5": 1,
                                     "wrj1": -1, "wrj2": -1}
        elif self._hand_prefix == "rh":
            self.command_sign_map = {"ffj1": 1, "ffj2": 1, "ffj3": -1, "ffj4": 1,
                                     "mfj1": 1, "mfj2": 1, "mfj3": -1, "mfj4": 1,
                                     "rfj1": 1, "rfj2": 1, "rfj3": -1, "rfj4": 1,
                                     "lfj1": -1, "lfj2": -1, "lfj3": 1, "lfj4": -1, "lfj5": -1,
                                     "thj1": -1, "thj2": -1, "thj3": 1, "thj4": -1, "thj5": -1,
                                     "wrj1": -1, "wrj2": 1}

    def _init_map_finger_joints(self):
        fingers_to_joint_map = OrderedDict()
        for joint in self._joint_msg.name:  # pylint: disable=E1101
            finger_name = joint[3:-2]
            if self._hand_prefix in joint:
                if finger_name not in fingers_to_joint_map:
                    fingers_to_joint_map[finger_name] = []
                fingers_to_joint_map[finger_name].append(joint[-2:])
        return fingers_to_joint_map

    def _init_controller_joints(self):
        controller_joints_names = []
        for finger, joint in self._fingers_to_joint_map.items():
            for joint_index in joint:
                # deal with different convention sensor/controllers due to coupled joints
                if finger not in FINGERS_WITHOUT_COUPLED_JOINTS:
                    if joint_index in COUPLED_JOINTS:
                        joint_index = "J0"
                full_joint_name = finger + joint_index
                controller_joints_names.append(full_joint_name.lower())
        return controller_joints_names

    def _init_finger_objects(self, fingers_to_test=None):
        fingers_to_check = []
        if not fingers_to_test:
            fingers_to_test = self._fingers_to_test

        for i, finger in enumerate(self._fingers_to_joint_map):
            if finger in fingers_to_test:
                fingers_to_check.append(Finger(self._hand_prefix, finger.lower()))
                for joint_index in self._fingers_to_joint_map[finger]:
                    fingers_to_check[i].joints_dict[joint_index] = Joint(self._hand_prefix, finger.lower(),
                                                                         joint_index.lower())

        fingers_to_check.sort(reverse=False, key=lambda x: x._get_sorting_value())
        return fingers_to_check

    def _init_raw_sensor_data_list(self):
        raw_sensor_names_list = []
        if "FF" in self._fingers_to_joint_map:
            for joint_index in self._fingers_to_joint_map["FF"]:
                name_to_append = self._hand_prefix + "_FF" + joint_index
                raw_sensor_names_list.append(name_to_append.lower())
        if "MF" in self._fingers_to_joint_map:
            for joint_index in self._fingers_to_joint_map["MF"]:
                name_to_append = self._hand_prefix + "_MF" + joint_index
                raw_sensor_names_list.append(name_to_append.lower())
        if "RF" in self._fingers_to_joint_map:
            for joint_index in self._fingers_to_joint_map["RF"]:
                name_to_append = self._hand_prefix + "_RF" + joint_index
                raw_sensor_names_list.append(name_to_append.lower())
        if "LF" in self._fingers_to_joint_map:
            for joint_index in self._fingers_to_joint_map["LF"]:
                name_to_append = self._hand_prefix + "_LF" + joint_index
                raw_sensor_names_list.append(name_to_append.lower())
        if "TH" in self._fingers_to_joint_map:
            for joint_index in self._fingers_to_joint_map["TH"]:
                if joint_index == "J5":
                    for iteration in range(0, 2):
                        name_to_append = self._hand_prefix + "_TH" + joint_index + f"_{iteration}"
                        raw_sensor_names_list.append(name_to_append.lower())
                else:
                    name_to_append = self._hand_prefix + "_TH" + joint_index
                    raw_sensor_names_list.append(name_to_append.lower())
        if "WR" in self._fingers_to_joint_map:
            for joint_index in self._fingers_to_joint_map["WR"]:
                if joint_index == "J1":
                    for iteration in range(0, 2):
                        name_to_append = self._hand_prefix + "_WR" + joint_index + f"_{iteration}"
                        raw_sensor_names_list.append(name_to_append.lower())
                else:
                    name_to_append = self._hand_prefix + "_WR" + joint_index
                    raw_sensor_names_list.append(name_to_append.lower())
        return raw_sensor_names_list

    def _joint_states_callback(self, sensor_msg):
        joint_dict = {}
        for name, position in zip(sensor_msg.name, sensor_msg.position):
            joint_dict.update({name.lower(): position})

        for finger in self.fingers_to_check:
            for joint in finger.joints_dict.values():
                joint.set_current_position(joint_dict[joint.joint_name])

    def _raw_data_sensor_callback(self, ethercat_data):
        for i in range(len(self._raw_sensor_names_list)):
            self._raw_sensor_data_map[self._raw_sensor_names_list[i]] = ethercat_data.sensors[i]

        for finger in self.fingers_to_check:
            for joint in finger.joints_dict.values():
                raw_sensor_data_list = []
                if joint.joint_name == self._hand_prefix + "_thj5":
                    raw_sensor_data_list.append(self._raw_sensor_data_map[self._hand_prefix + '_thj5_0'])
                    raw_sensor_data_list.append(self._raw_sensor_data_map[self._hand_prefix + '_thj5_1'])
                elif joint.joint_name == self._hand_prefix + "_wrj1":
                    raw_sensor_data_list.append(self._raw_sensor_data_map[self._hand_prefix + '_wrj1_0'])
                    raw_sensor_data_list.append(self._raw_sensor_data_map[self._hand_prefix + '_wrj1_1'])
                else:
                    raw_sensor_data_list.insert(0, self._raw_sensor_data_map[joint.joint_name])
                joint.set_raw_sensor_data(raw_sensor_data_list)

    def switch_controller_mode(self, control_type):
        if control_type == "trajectory":
            rospy.loginfo("Changing trajectory controllers to RUN")
            self.ctrl_helper.change_trajectory_ctrl("run")
        elif control_type in ("position", "effort"):
            self.ctrl_helper.change_trajectory_ctrl("stop")
            change_type_msg = ChangeControlType()
            change_type_msg.control_type = ControlType.PWM
            self.ctrl_helper.change_force_ctrl_type(change_type_msg)
            rospy.loginfo(f"Changing controllers to: {control_type}")
            self.ctrl_helper.change_hand_ctrl(control_type)

    def drive_joint_to_position(self, joint, command):
        self.switch_controller_mode("position")
        joint.move_joint(command, "position")
        self.switch_controller_mode("effort")

    def drive_joint_with_pwm(self, joint, command, duration, rate):  # pylint: disable=R0201
        now = rospy.Time.now()
        while rospy.Time.now() < now + rospy.Duration(duration):
            joint.move_joint(command, "effort")
            rate.sleep()
        joint.move_joint(0, "effort")

    """
        Get the result of the test
        @return: Dictionary containing the result of the test
    """
    def get_result(self):
        return self._result

    """
        Moves tested fingers into 0 degree joint anglees in position control mode.
    """
    def move_fingers_to_start_position(self):
        self.switch_controller_mode('position')
        for finger in self.fingers_to_check:
            finger.move_finger(0, 'position')
            rospy.logwarn(f"Moving to start {finger.finger_name}")

    """
        Moves the finger to the left/right joint limit of J4
        @param finger_object: Finger object indicating which finger to move
        @param side: String defining the side, 'right' or 'left'
    """
    def move_finger_to_side(self, finger_object, side):
        angle = math.radians(-20) if side == 'right' else math.radians(20)
        angle *= self._side_sign_map[finger_object.finger_name]
        if "J4" in finger_object.joints_dict:
            finger_object.joints_dict['J4'].move_joint(angle, 'position')
            rospy.logwarn(f"moving {finger_object.finger_name} to {side}")

    """
        Checks if the test execution result passed
        @return Bool value
    """
    def has_passed(self):
        raise NotImplementedError("The function 'has_passed' must be implemented")

    def has_single_passed(self, name, value):
        raise NotImplementedError("The function 'has_single_passed' must be implemented")

    def stop_test(self):
        self._stopped_execution = True

    def is_stopped(self):
        return self._stopped_execution

    def get_name(self):
        return self._name
