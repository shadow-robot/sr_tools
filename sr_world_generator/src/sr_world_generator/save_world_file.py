#!/usr/bin/env python3

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

import re
import os
import subprocess
import rospy
import rospkg
from gazebo_msgs.msg import ModelStates
from tf.transformations import euler_from_quaternion


class GazeboWorldSaver:
    def __init__(self, gazebo_generated_world_file_path=None, output_world_file_path=None):

        self.model_and_pose = {}
        if gazebo_generated_world_file_path is None:
            self.gazebo_generated_world_file_path = rospy.get_param('~gazebo_generated_world_file_path')
        else:
            self.gazebo_generated_world_file_path = gazebo_generated_world_file_path

        if output_world_file_path is None:
            output_world_file_name = rospy.get_param('~output_world_file_name', 'new_world') + '.world'
            description_path = rospkg.RosPack().get_path('sr_description_common')
            self.output_world_file_path = description_path + '/worlds/' + output_world_file_name
        else:
            self.output_world_file_path = output_world_file_path

        self.config_path = rospkg.RosPack().get_path('sr_world_generator') + '/config'

        self._check_if_world_file_exists()
        self._start_gazebo_with_newly_created_world()
        self._get_gazebo_models_states()
        self._extract_model_data_from_msg()
        self._initiate_world_file()
        self._save_lighting_config_to_world_file()
        self._save_models_to_world_file()
        self._save_physics_config_to_world_file()
        self._finish_up_world_file()
        self._clean_exit()

    def _check_if_world_file_exists(self):
        if not os.path.isfile(self.gazebo_generated_world_file_path):
            raise IOError("Gazebo generated world file does not exist!")

    def _start_gazebo_with_newly_created_world(self):
        # pylint: disable=R1732
        self.process = subprocess.Popen(['xterm -e roslaunch sr_world_generator \
                                        create_world_template.launch gui:=false \
                                        scene:=true world:={}'.format(self.gazebo_generated_world_file_path)],
                                        shell=True)

    def _clean_exit(self):
        rospy.loginfo("World saved!")
        self.process.kill()

    def _get_gazebo_models_states(self):
        self.gazebo_model_states_msg = rospy.wait_for_message('/gazebo/model_states', ModelStates)

    def _extract_model_data_from_msg(self):
        for model_name, pose in zip(self.gazebo_model_states_msg.name, self.gazebo_model_states_msg.pose): # pylint: disable=E1101
            if model_name == 'ursr':
                continue
            position_as_list = [pose.position.x, pose.position.y, pose.position.z]
            orientation_as_list = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
            orientation_as_list_euler = list(euler_from_quaternion(orientation_as_list))
            pose_as_list = position_as_list + orientation_as_list_euler
            self.model_and_pose[model_name] = ' '.join(str(val) for val in pose_as_list)

    def _save_models_to_world_file(self):
        all_objects_string = ''
        for key, value in self.model_and_pose.items():
            all_objects_string += '    <include>\n'
            all_objects_string += '      <uri>model://' + re.sub(r'_\d+$', '', key) + '</uri>\n'
            all_objects_string += '      <static>true</static>\n'
            all_objects_string += '      <name>' + key + '</name>\n'
            all_objects_string += '      <pose>' + value + '</pose>\n'
            all_objects_string += '    </include>\n'
        self._save_to_world_file(all_objects_string)

    def _save_lighting_config_to_world_file(self):
        with open(self.config_path + '/gazebo_light_string', 'r') as myfile:
            data = myfile.readlines()
        data = ''.join(data) + '\n'
        self._save_to_world_file(data)

    def _save_physics_config_to_world_file(self):
        with open(self.config_path + '/gazebo_physics_string', 'r') as myfile:
            data = myfile.readlines()
        data = ''.join(data) + '\n'
        self._save_to_world_file(data)

    def _initiate_world_file(self):
        self._remove_output_file_if_exists()
        leading_string = '<?xml version="1.0" ?>\n' + '<sdf version="1.4">\n' + '  <world name="default">\n'
        self._save_to_world_file(leading_string)

    def _finish_up_world_file(self):
        trailing_string = '  </world>\n' + '</sdf>'
        self._save_to_world_file(trailing_string)

    def _save_to_world_file(self, string):
        with open(self.output_world_file_path, 'a') as myfile:
            myfile.write(string)

    def _remove_output_file_if_exists(self):
        full_file_path = self.output_world_file_path
        if os.path.isfile(full_file_path):
            os.remove(full_file_path)


if __name__ == '__main__':
    rospy.init_node('save_gazebo_world_file', anonymous=True)
    gws = GazeboWorldSaver()
