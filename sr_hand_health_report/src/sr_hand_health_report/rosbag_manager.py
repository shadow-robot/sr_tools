#!/usr/bin/env python3

# Copyright 2020-2022 Shadow Robot Company Ltd.
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


from __future__ import absolute_import
import rospy
import os
import datetime
import subprocess


class RosbagManager(object):
    def __init__(self, log_test_directory):
        self._rosbag_proc = None
        self._log_test_directory = log_test_directory
        self._start_time = rospy.Time.now()

    def start_log(self):
        if not os.path.exists(self._log_test_directory):
            os.mkdir(self._log_test_directory)
            f = open("{}/README.txt".format(self._log_test_directory), "w+")
            f.write("Test")
            f.close()
        os.system("rosparam dump {}/param_dump.yaml".format(self._log_test_directory))

    def record_bag(self, log_test_directory, bag_name):
        """
        Start data logging in folder and with test description
        """
        rospy.loginfo("Loggin data in {} in {} bag file".format(log_test_directory, bag_name))

        os.system("killall rosbag")
        os.system("killall rostopic")
        try:
            self._rosbag_proc = subprocess.Popen(['rosbag record  \
                -O {}/{} -a __name:=bag_node'.format(log_test_directory, bag_name)], shell=True)
            rospy.sleep(2.0)  # give time to record to start
        except OSError as e:
            rospy.logerr("Could not start rosbag record")
            self._rosbag_proc.kill()

    def play_bag(self, log_test_directory, bag_name):
        """
        Start playing a rosbg in a given folder
        """
        rospy.loginfo("Playing rosbag {}/{}".format(log_test_directory, bag_name))

        try:
            self._rosbag_proc = subprocess.Popen(['rosbag play -q {}/{} \
                --clock'.format(log_test_directory, bag_name)], shell=True)
        except OSError as e:
            rospy.logerr("Could not play rosbag")
            self._rosbag_proc.kill()

    def stop_bag(self):
        """
        Stops the currently running rosbag process.
        """
        if self._rosbag_proc:
            os.system("rosnode kill bag_node")
