#!/usr/bin/env python3

# Copyright 2019 Shadow Robot Company Ltd.
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
import sys
from python_qt_binding.QtGui import *
from python_qt_binding.QtCore import *
from qt_gui.plugin import Plugin
from python_qt_binding import loadUi

try:
    from python_qt_binding.QtWidgets import *
except ImportError:
    pass

from sr_world_generator.save_world_file import GazeboWorldSaver

import signal
import rospy
import os
import rospkg
import subprocess


class SrWorldGeneratorGui(Plugin):
    def __init__(self, context):
        super(SrWorldGeneratorGui, self).__init__(context)
        self.setObjectName("SrWorldGeneratorGui")
        self._widget = QWidget()

        ui_file = os.path.join(rospkg.RosPack().get_path(
            'sr_world_generator'), 'uis', 'world_generator.ui')

        loadUi(ui_file, self._widget)
        if __name__ != "__main__":
            context.add_widget(self._widget)

        self._widget.setWindowTitle("Gazebo World Generator")
        self.init_widget_children()

        self.open_gazebo_button.clicked.connect(self.start_gazebo_process)
        self.close_gazebo_button.clicked.connect(self.stop_gazebo_process)
        self.world_browse_button.clicked.connect(self.get_world_file_path)
        self.gazebo_generated_file_browse.clicked.connect(self.get_world_file_to_transform_path)
        self.transform_world_file_button.clicked.connect(self.transform_world_file)

        self.empty_world_yes_radio.clicked.connect(self.disable_world_path)
        self.empty_world_no_radio.clicked.connect(self.enable_world_path)

    def init_widget_children(self):
        self.open_gazebo_button = self._widget.findChild(QPushButton, "open_gazebo_button")
        self.close_gazebo_button = self._widget.findChild(QPushButton, "close_gazebo_button")
        self.world_browse_button = self._widget.findChild(QPushButton, "world_browse_button")
        self.gazebo_generated_file_browse = self._widget.findChild(QPushButton, "gazebo_generated_file_browse")
        self.transform_world_file_button = self._widget.findChild(QPushButton, "transform_world_file_button")

        self.start_home_yes_radio = self._widget.findChild(QRadioButton, "start_home_yes_radio")
        self.start_home_no_radio = self._widget.findChild(QRadioButton, "start_home_no_radio")
        self.empty_world_yes_radio = self._widget.findChild(QRadioButton, "empty_world_yes_radio")
        self.empty_world_no_radio = self._widget.findChild(QRadioButton, "empty_world_no_radio")

        self.world_line_edit = self._widget.findChild(QLineEdit, "world_line_edit")
        self.initial_z_line_edit = self._widget.findChild(QLineEdit, "initial_z_line_edit")
        self.gazebo_generated_world_path_line_edit = self._widget.findChild(QLineEdit,
                                                                            "gazebo_generated_world_path_line_edit")

        self.transform_file_group_box = self._widget.findChild(QGroupBox, "transform_file_group_box")

    def destruct(self):
        self._widget = None
        rospy.loginfo("Closing gazebo world generator")

    def start_gazebo_process(self):
        initial_z = self.initial_z_line_edit.displayText()
        is_robot_starting_home = self.start_home_yes_radio.isChecked()
        is_starting_with_empty_world = self.empty_world_yes_radio.isChecked()

        gazebo_start_command = 'xterm -e roslaunch sr_world_generator' + \
                               ' create_world_template.launch start_home:={} '.format(is_robot_starting_home) + \
                               'scene:={} initial_z:={}'.format(not is_starting_with_empty_world, initial_z)

        if not is_starting_with_empty_world:
            world_file_path = self.world_line_edit.displayText()
            if not os.path.isfile(world_file_path):
                self.throw_warning_dialog("File does not exist!")
                return
            gazebo_start_command += ' world:={}'.format(world_file_path)

        self.open_gazebo_button.setEnabled(False)
        self.close_gazebo_button.setEnabled(True)
        self.transform_file_group_box.setEnabled(False)

        self.process = subprocess.Popen([gazebo_start_command],
                                        shell=True)

    def stop_gazebo_process(self):
        self.process.kill()

        self.open_gazebo_button.setEnabled(True)
        self.close_gazebo_button.setEnabled(False)
        self.transform_file_group_box.setEnabled(True)

    def transform_world_file(self):
        worlds_path = rospkg.RosPack().get_path('sr_description_common') + '/worlds/'
        output_file_path = QFileDialog.getSaveFileName(self._widget, 'Save File',
                                                       worlds_path + 'new_world.world',
                                                       'World Files (*.world)')
        gazebo_generated_world_file_path = self.gazebo_generated_world_path_line_edit.displayText()
        if not os.path.isfile(gazebo_generated_world_file_path):
            self.throw_warning_dialog("File chosen to be transformed does not exist!")
            return
        gws = GazeboWorldSaver(gazebo_generated_world_file_path, output_file_path[0])

    def enable_world_path(self):
        self.world_line_edit.setEnabled(True)
        self.world_browse_button.setEnabled(True)

    def disable_world_path(self):
        self.world_line_edit.setEnabled(False)
        self.world_browse_button.setEnabled(False)

    def get_world_file_path(self):
        self.world_line_edit.clear()
        chosen_path = self.get_file_path()
        self.world_line_edit.setText(chosen_path)

    def get_world_file_to_transform_path(self):
        self.gazebo_generated_world_path_line_edit.clear()
        chosen_path = self.get_file_path()
        self.gazebo_generated_world_path_line_edit.setText(chosen_path)

    def get_file_path(self):
        chosen_path = QFileDialog.getOpenFileName(self._widget, 'Open file', "")
        return chosen_path[0]

    def throw_warning_dialog(self, message):
        msg = QMessageBox()
        msg.setIcon(QMessageBox.Warning)
        msg.setText(message)
        msg.setWindowTitle("Warning!")
        msg.setStandardButtons(QMessageBox.Ok)
        msg.exec_()


if __name__ == "__main__":
    rospy.init_node("sr_gazebo_world_generator")
    app = QApplication(sys.argv)
    planner_benchmarking_gui = SrWorldGeneratorGui(None)
    planner_benchmarking_gui._widget.show()
    signal.signal(signal.SIGINT, signal.SIG_DFL)
    sys.exit(app.exec_())
