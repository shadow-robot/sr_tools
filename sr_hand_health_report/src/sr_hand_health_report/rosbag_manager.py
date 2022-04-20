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
import os
import subprocess
import rospy


class RosbagManager:
    def __init__(self, log_test_directory):
        self._rosbag_proc = None
        self._log_test_directory = log_test_directory
        self._start_time = rospy.Time.now()

    def start_log(self):
        if not os.path.exists(self._log_test_directory):
            os.mkdir(self._log_test_directory)
            with open("{}/README.txt".format(self._log_test_directory), "w+", encoding="ASCII") as log:
                log.write("Test")
        os.system(f"rosparam dump {self._log_test_directory}/param_dump.yaml")

    def record_bag(self, log_test_directory, bag_name):
        """
        Start data logging in folder and with test description
        """
        rospy.loginfo(f"Loggin data in {log_test_directory} in {bag_name} bag file")

        os.system("killall rosbag")
        os.system("killall rostopic")
        try:
            self._rosbag_proc = subprocess.Popen([f'rosbag record  \
                -O {log_test_directory}/{bag_name} -a __name:=bag_node'], shell=True)  # pylint: disable=R1732
            rospy.sleep(2.0)  # give time to record to start
        except OSError:
            rospy.logerr("Could not start rosbag record")
            self._rosbag_proc.kill()

    def play_bag(self, log_test_directory, bag_name):
        """
        Start playing a rosbg in a given folder
        """
        rospy.loginfo(f"Playing rosbag {log_test_directory}/{bag_name}")

        try:
            self._rosbag_proc = subprocess.Popen([f'rosbag play -q {log_test_directory}/{bag_name} \
                --clock'], shell=True)  # pylint: disable=R1732
        except OSError:
            rospy.logerr("Could not play rosbag")
            self._rosbag_proc.kill()

    def stop_bag(self):
        """
        Stops the currently running rosbag process.
        """
        if self._rosbag_proc:
            os.system("rosnode kill bag_node")
