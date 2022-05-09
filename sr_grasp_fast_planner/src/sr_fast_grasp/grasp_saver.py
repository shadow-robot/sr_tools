#!/usr/bin/env python 3

# Copyright 2019, 2022 Shadow Robot Company Ltd.
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

import sys
import rospy
from moveit_msgs.srv import SaveRobotStateToWarehouse as SaveState
from moveit_msgs.msg import RobotState
from sensor_msgs.msg import JointState


class GraspSaver:
    def __init__(self, name):
        self.__group_name = "right_hand"  # pylint: disable=W0238
        self.__save = rospy.ServiceProxy(
            '/moveit_warehouse_services/save_robot_state', SaveState)
        self.__js_subscriber = rospy.Subscriber("joint_states",  # pylint: disable=W0238
                                                JointState,
                                                self.__js_cb)
        self.__js = None
        self.__done = False
        self.__name = name

    def __js_cb(self, js):  # pylint: disable=C0103
        self.__js = js

    def __save_out(self):
        robotstate = RobotState()
        robotstate.joint_state = self.__js
        self.__save(self.__name, "", robotstate)

    def spin(self):
        while not self.__done and not rospy.is_shutdown():
            if self.__js is not None:
                self.__save_out()
                self.__done = True
            else:
                rospy.sleep(.1)


if __name__ == "__main__":
    rospy.init_node("grasp_saver")
    if len(sys.argv) <= 1 or sys.argv[1] == "":
        rospy.logerr("You didn't enter a name.")
        sys.exit(-1)
    gs = GraspSaver(sys.argv[1])
    gs.spin()
