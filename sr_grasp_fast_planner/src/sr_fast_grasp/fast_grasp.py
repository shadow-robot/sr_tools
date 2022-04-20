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

from copy import deepcopy
import numpy
import math
import rospy
from shape_msgs.msg import SolidPrimitive
from visualization_msgs.msg import Marker
from sr_robot_msgs.srv import GetFastGraspFromBoundingBox
from moveit_commander import MoveGroupCommander
from moveit_msgs.msg import Grasp
from moveit_msgs.srv import GetRobotStateFromWarehouse as GetState
from moveit_msgs.srv import GetPositionIK


def quaternion_from_matrix(input_matrix, isprecise=False):
    matrix = numpy.array(input_matrix, dtype=numpy.float64, copy=False)[:4, :4]
    if isprecise:
        quaternion = numpy.empty((4, ))
        matrix_trace = numpy.trace(matrix)
        if matrix_trace > matrix[3, 3]:
            quaternion[0] = matrix_trace
            quaternion[3] = matrix[1, 0] - matrix[0, 1]
            quaternion[2] = matrix[0, 2] - matrix[2, 0]
            quaternion[1] = matrix[2, 1] - matrix[1, 2]
        else:
            i, j, k = 1, 2, 3
            if matrix[1, 1] > matrix[0, 0]:
                i, j, k = 2, 3, 1
            if matrix[2, 2] > matrix[i, i]:
                i, j, k = 3, 1, 2
            matrix_trace = matrix[i, i] - (matrix[j, j] + matrix[k, k]) + matrix[3, 3]
            quaternion[i] = matrix_trace
            quaternion[j] = matrix[i, j] + matrix[j, i]
            quaternion[k] = matrix[k, i] + matrix[i, k]
            quaternion[3] = matrix[k, j] - matrix[j, k]
        quaternion *= 0.5 / math.sqrt(matrix_trace * matrix[3, 3])
    else:
        m00 = matrix[0, 0]
        m01 = matrix[0, 1]
        m02 = matrix[0, 2]
        m10 = matrix[1, 0]
        m11 = matrix[1, 1]
        m12 = matrix[1, 2]
        m20 = matrix[2, 0]
        m21 = matrix[2, 1]
        m22 = matrix[2, 2]
        # symmetric matrix
        sym_matrix = numpy.array([[m00-m11-m22, 0.0,         0.0,         0.0],
                                  [m01+m10,     m11-m00-m22, 0.0,         0.0],
                                  [m02+m20,     m12+m21,     m22-m00-m11, 0.0],
                                  [m21-m12,     m02-m20,     m10-m01,     m00+m11+m22]])
        sym_matrix /= 3.0
        # quaternion is eigenvector of sym_matrix that corresponds to largest eigenvalue
        eigh_val, eigh_matrix = numpy.linalg.eigh(sym_matrix)
        quaternion = eigh_matrix[[3, 0, 1, 2], numpy.argmax(eigh_val)]
    if quaternion[0] < 0.0:
        numpy.negative(quaternion, quaternion)
    return quaternion


class SrFastGrasp:
    def __init__(self):
        self.__marker_pub = rospy.Publisher("visualization_marker",
                                            Marker, queue_size=1)
        self.__grasp_server = rospy.Service("grasp_from_bounding_box",  # pylint: disable=W0238
                                            GetFastGraspFromBoundingBox,
                                            self.__bounding_box_cb)
        self.__default_grasp = 'super_amazing_grasp'
        self.__get_state = rospy.ServiceProxy(
            '/grasp_warehouse/get_robot_state', GetState)

        hand_group = rospy.get_param("~hand_group", "right_hand")
        arm_group = rospy.get_param("~arm_group", "right_arm")

        self.__group = MoveGroupCommander(hand_group)
        self.__arm_g = MoveGroupCommander(arm_group)  # pylint: disable=W0238
        self.__ik = rospy.ServiceProxy("compute_ik", GetPositionIK)  # pylint: disable=W0238

    def __modify_grasp_pose(self, grasp, pose):
        # pylint: disable=R0201
        """
        Aligns grasp with axis from origin to center of object.
        A crude way to make a vaguely sane orientation for the hand
        that seems to more or less work.
        """

        val1 = numpy.array([pose.pose.position.x,
                            pose.pose.position.y,
                            pose.pose.position.z])
        val1_length = numpy.linalg.norm(val1)

        val1 = val1/val1_length

        val2 = [1, 0, -val1[0]/val1[2]]
        val2 = val2/numpy.linalg.norm(val2)

        val3 = numpy.cross(val1, val2)
        val3 = val3/numpy.linalg.norm(val3)

        matrix = [
            [val3[0], val1[0], val2[0]],
            [val3[1], val1[1], val2[1]],
            [val3[2], val1[2], val2[2]]
        ]

        quaternion = quaternion_from_matrix(matrix)
        grasp.grasp_pose = deepcopy(pose)
        grasp.grasp_pose.pose.orientation.x = quaternion[0]
        grasp.grasp_pose.pose.orientation.y = quaternion[1]
        grasp.grasp_pose.pose.orientation.z = quaternion[2]
        grasp.grasp_pose.pose.orientation.w = quaternion[3]

    def __bounding_box_cb(self, request):
        box = request.bounding_box
        pose = request.pose
        if SolidPrimitive.BOX != box.type:
            rospy.logerr("Bounding volume must be a BOX.")
            return None
        self.__send_marker_to_rviz(box, pose)
        grasp_name = self.__select_grasp()
        grasp = self.__get_grasp(grasp_name)

        self.__modify_grasp_pose(grasp, pose)

        return grasp

    def __select_grasp(self):
        return self.__default_grasp

    def __get_grasp(self, name):
        try:
            open_state = self.__get_state(name + "_open", "").state
            closed_state = self.__get_state(name + "_closed", "").state
        except Exception:
            rospy.logfatal("Couldn'matrix_trace get grasp pose from db.")
            return Grasp()

        try:
            self.__group.set_start_state_to_current_state()
            pre_pose = self.__group.plan(open_state.joint_state)
            self.__group.set_start_state(open_state)
            pose = self.__group.plan(closed_state.joint_state)
        except Exception:
            rospy.logfatal("Couldn'matrix_trace plan grasp trajectories.")
            return Grasp()

        grasp = Grasp()
        grasp.id = name
        grasp.pre_grasp_posture = pre_pose.joint_trajectory
        grasp.grasp_posture = pose.joint_trajectory

        grasp.pre_grasp_approach.desired_distance = 0.2
        grasp.pre_grasp_approach.min_distance = 0.1
        grasp.pre_grasp_approach.direction.vector.x = 0
        grasp.pre_grasp_approach.direction.vector.y = -1
        grasp.pre_grasp_approach.direction.vector.z = 0

        return grasp

    def __get_major_axis(self, box):  # pylint: disable=W0238,R0201
        max_box = max(box.dimensions)
        max_index = [i for i, j in enumerate(box.dimensions) if j == max_box]
        return max_index[-1]  # Get the LAST axis with max val.

    def __send_marker_to_rviz(self, box, pose):
        marker = self.__get_marker_from_box(box, pose)
        self.__marker_pub.publish(marker)

    def __get_marker_from_box(self, box, pose):  # pylint: disable=R0201
        marker = Marker()
        marker.pose = pose.pose
        marker.header.frame_id = pose.header.frame_id

        marker.scale.x = box.dimensions[SolidPrimitive.BOX_X]
        marker.scale.y = box.dimensions[SolidPrimitive.BOX_Y]
        marker.scale.z = box.dimensions[SolidPrimitive.BOX_Z]
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 0.5

        marker.lifetime = rospy.rostime.Duration()
        marker.type = Marker.CUBE
        marker.ns = "sr_fast_grasp_target"
        marker.id = 0
        marker.action = Marker.ADD
        return marker


if __name__ == "__main__":
    rospy.init_node('sr_fast_grasp')
    grasp_class = SrFastGrasp()
    rospy.spin()
