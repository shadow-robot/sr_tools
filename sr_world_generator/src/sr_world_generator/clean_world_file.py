#!/usr/bin/env python3

# Copyright 2022 Shadow Robot Company Ltd.
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

import xml.etree.ElementTree as ET
import rospy


class WorldFileCleaner:
    """
    This class is used to clean up a Gazebo world file by removing named models and state information.
    This is useful when you want to save a world file that has been modified by Gazebo, but you don't want to save the
    state information or any models (e.g. robots) that you don't want to be in the world file.
    """
    _xml_tree: ET.ElementTree = None

    def __init__(self, dry_run: bool = False):
        self._dry_run = dry_run

    def load_world_file(self, world_file_path: str):
        """
        Loads the world file from the specified path.

        Args:
            world_file_path: The path to the world file to load.
        """
        rospy.loginfo(f'Loading world file from {world_file_path}')
        self._xml_tree = ET.parse(world_file_path)

    def remove_models(self, removed_model_names: "list[str]"):
        """
        Removes any models with names in the supplied list from the world file.

        Args:
            removed_model_names: A list of names models that will be removed.
        """
        xml_root = self._xml_tree.getroot()
        # Find all elements that have model elements as children; we need the parent reference for removal
        for parent in xml_root.findall('.//model/..'):
            # Find all model elements in the parent
            for model in parent.findall('model'):
                name = model.get('name')
                # If the model name is in the list of models to remove, remove it
                if name in removed_model_names:
                    rospy.loginfo(f'Removing model: {name}')
                    parent.remove(model)

    def remove_state(self):
        """
        Removes the state information from the world file.
        """
        xml_root = self._xml_tree.getroot()
        # Find all elements that have state elements as children; we need the parent reference for removal
        for parent in xml_root.findall('.//state/..'):
            for state in parent.findall('state'):
                rospy.loginfo('Removing state information.')
                parent.remove(state)

    def save_world_file(self, output_file_path: str):
        """
        Saves the world file to the specified path.

        Args:
            output_file_path: The path to save the world file to.
        """
        # Only save the file if we're not in dry run mode
        if not self._dry_run:
            rospy.loginfo(f'Saving world file to {output_file_path}')
            self._xml_tree.write(output_file_path)


if __name__ == '__main__':
    # Initialise the ROS node
    rospy.init_node('clean_gazebo_world_file', anonymous=True)
    # Get parameters from the ROS parameter server
    removed_model_names_param = rospy.get_param("~removed_model_names")
    input_world_file_path_param = rospy.get_param("~input_world_file_path")
    output_world_file_path_param = rospy.get_param("~output_world_file_path")
    dry_run_param = rospy.get_param("~dry_run")
    # Create a new WorldFileCleaner object
    world_file_cleaner = WorldFileCleaner(dry_run_param)
    # Load the world file, remove the models and state, and save the world file
    world_file_cleaner.load_world_file(input_world_file_path_param)
    world_file_cleaner.remove_models(removed_model_names_param)
    world_file_cleaner.remove_state()
    world_file_cleaner.save_world_file(output_world_file_path_param)
