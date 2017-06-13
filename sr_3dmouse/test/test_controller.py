#!/usr/bin/env python
'''
3d mouse tester module
'''
from __future__ import division
import unittest
import actionlib
import rospy
from moveit_msgs.msg import MoveGroupAction
from sensor_msgs.msg import Joy
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose
from sr_robot_commander.sr_arm_commander import SrArmCommander


PKG = 'mouse_test'


class MouseTester(unittest.TestCase):
    '''
    Main classmethod
    '''
    def setUp(self):
        # init
        rospy.init_node("mouse_tester", anonymous=True)
        client = actionlib.SimpleActionClient('move_group', MoveGroupAction)
        client.wait_for_server()
        rospy.sleep(15.)
        self.data = Joy()
        self.target_pose = PoseStamped()
        start_pose = Pose()
        self.new_pose = PoseStamped()
        # expected values
        self.expvals = [[[0.3, 0.0, 1.1, 0, 0.7, 0.05, 0.05],
                         [0.51, 0.25, 1.1, 0.7, 0.71, 0.05, 0.048],
                         [0.51, 0.26, 1.1, 0.7, 0.71, 0.05, 0.048]],
                        [[0.27, 0.25, 1.1, 0.7, 0.7, 0.05, 0.05],
                         [0.27, 0.35, 1.1, 0.7, 0.7, 0.05, 0.05],
                         [0.27, 0.35, 1.1, 0.7, 0.7, 0.05, 0.05]],
                        [[0.27, 0.25, 1.1, 0.7, 0.7, 0.05, 0.05],
                         [0.27, 0.25, 0.9, 0.7, 0.7, 0.05, 0.05],
                         [0.27, 0.25, 0.85, 0.7, 0.7, 0.05, 0.05]],
                        [[0.27, 0.25, 1.1, 0.7, 0.7, 0.05, 0.05],
                         [0.27, 0.25, 1.1, 0.69, 0.7, 0.05, 0.1],
                         [0.27, 0.25, 1, 0.7, 0.7, 0.05, 0.1]],
                        [[0.27, 0.25, 1.1, 0.7, 0.7, 0.05, 0.05],
                         [0.27, 0.25, 1.1, 0.69, 0.69, 0.1, 0.1],
                         [0.27, 0.25, 1.1, 0.69, 0.69, 0.1, 0.1]],
                        [[0.27, 0.25, 1.1, 0.7, 0.7, 0.05, 0.05],
                         [0.27, 0.25, 1.1, 0.7, 0.6, 0.05, 0.08],
                         [0.27, 0.25, 1.1, 0.7, 0.6, 0.05, 0.05]]]
        # initialize arm's position
        self._arm_commander = SrArmCommander(name="right_arm", set_ground=True)
        start_pose = self._arm_commander.get_current_pose()
        self.new_pose.header.stamp = rospy.get_rostime()
        self.new_pose.header.frame_id = "world"
        self.new_pose.pose = start_pose

        self.new_pose.pose.position.x = 0.3
        self.new_pose.pose.position.y = 0.25
        self.new_pose.pose.position.z = 1.1
        self.new_pose.pose.orientation.x = 0.7
        self.new_pose.pose.orientation.y = 0.7
        self.new_pose.pose.orientation.z = 0
        self.new_pose.pose.orientation.w = 0
        self.sub = rospy.Subscriber("command", PoseStamped, self.comparecall)
        self.pub_arm = rospy.Publisher("command", PoseStamped, queue_size=1)
        self.pub = rospy.Publisher("/spacenav/joy", Joy, queue_size=1)
        # initialise joy publisher
        self.data.buttons = [0, 0]
        self.data.axes = [0, 0, 0, 0, 0, 0]
        self.reset_joy = Joy()
        self.reset_joy = self.data
        self.pub.publish(self.data)
        # move arm to initial position
        self._arm_commander.move_to_pose_value_target_unsafe(self.new_pose,
                                                             avoid_collisions=True,
                                                             wait=False)
        self.pub_arm.publish(self.new_pose)
        # wait for two seconds to finish initialization
        rospy.sleep(2)
        self.tester()

    def tester(self):
        "publishing data"

        rate = rospy.Rate(1)
        for i in range(0, 6):
            self.data.axes = [0, 0, 0, 0, 0, 0]
            self.data.axes[i] = -0.08
            for j in range(0, 3):
                self.pub.publish(self.data)
                self.data.axes[i] = self.data.axes[i] + 0.08
                rate.sleep()
                self.assertAlmostEqual(self.target_pose.pose.position.x,
                                       abs(self.expvals[i][j][0]), msg="poseX failed", delta=2)
                self.assertAlmostEqual(self.target_pose.pose.position.y,
                                       abs(self.expvals[i][j][1]), msg="poseY failed", delta=2)
                self.assertAlmostEqual(self.target_pose.pose.position.z,
                                       abs(self.expvals[i][j][2]), msg="poseZ failed", delta=2)
                self.assertAlmostEqual(self.target_pose.pose.orientation.x,
                                       abs(self.expvals[i][j][3]), msg="rotX failed", delta=2)
                self.assertAlmostEqual(self.target_pose.pose.orientation.y,
                                       abs(self.expvals[i][j][4]), msg="rotY failed", delta=2)
                self.assertAlmostEqual(self.target_pose.pose.orientation.z,
                                       abs(self.expvals[i][j][5]), msg="rotZ failed", delta=2)
                self.assertAlmostEqual(self.target_pose.pose.orientation.w,
                                       abs(self.expvals[i][j][6]), msg="rotW failed", delta=2)
        # reset joystic
        self.reset_joy.axes = [0, 0, 0, 0, 0, 0]
        self.pub.publish(self.reset_joy)

    def comparecall(self, data):
        '''
        checks the output of the controller
        '''
        self.target_pose = data

if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG, "mouse_test", MouseTester)
