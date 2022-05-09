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

from builtins import round, input
import csv
import numpy
import rospy
from diagnostic_msgs.msg import DiagnosticArray


class CalibGauges():
    def __init__(self):
        self.initial_weight = rospy.get_param('~initial_weight')
        self.incremental_weight = rospy.get_param('~incremental_weight')
        self.final_weight = rospy.get_param('~final_weight')
        self.motor_id = 0
        self.number_of_tests = (self.final_weight-self.initial_weight)/self.incremental_weight
        self.strain_gauges_id = 0
        self.measurement = []
        self.headline_0 = []
        self.temp_0 = []
        self.headline_1 = []
        self.temp_1 = []
        self.sub = None

    def data_callback(self, data):
        # iterate over the DiagnosticStatus message
        for i, ds_value in enumerate(data.status):
            fields = {}
            # iterate over values array and store them in a dictionary with index and relative value
            for _, value in enumerate(ds_value.values):
                fields[i, value.key] = value.value
            # iterate over dictionary elements to search the wanted motor_id and the relative strain_gauge value
            for index, _ in list(fields.items()):
                if fields.get((index[0], 'Motor ID')) == f"{self.motor_id}":
                    if self.strain_gauges_id == 0:
                        # store measurement in array
                        self.measurement.append(fields.get((index[0], 'Strain Gauge Left')))
                    else:
                        # store measurement in array
                        self.measurement.append(fields.get((index[0], 'Strain Gauge Right')))

    def run_test(self):
        self.motor_id = input("insert motor id: ")
        for self.strain_gauges_id in range(0, 2):
            self.initial_weight = 0
            for _ in range(0, self.number_of_tests):
                self.initial_weight += self.incremental_weight
                print(f"Apply {self.initial_weight} g to strain_gauges {self.strain_gauges_id}")
                input("Press enter when you are done...")
                # Subscribe diagnostic topic to collect data
                self.sub = rospy.Subscriber("/diagnostics_agg", DiagnosticArray, self.data_callback)
                print("Measuring...")
                rospy.sleep(5.0)  # give time to collect measurements
                self.sub.unregister()  # stop subscribing topic
                self.update_csv_file(self.measurement, self.strain_gauges_id)  # write measurements into csv file
                self.measurement = []  # empty measurement array

    def update_csv_file(self, measurement, strain_gauges_id):
        # pylint: disable=R1732
        if strain_gauges_id == 0:
            # create csv to write
            csv_output = csv.writer(open(f"Motor_{self.motor_id}_strain_gauge_{strain_gauges_id}.csv",
                                    "wb", encoding="ASCII"), lineterminator='\n')
            self.headline_0.append(self.initial_weight)
            csv_output.writerow(self.headline_0)   # write headline row with weight values
            measurement = list(map(int, measurement))  # cast values to int
            self.temp_0.append(round(numpy.average(measurement)))  # compute values average
            csv_output.writerow(self.temp_0)  # write measurements

        elif strain_gauges_id == 1:
            # create csv to write
            csv_output = csv.writer(open(f"Motor_{self.motor_id}_strain_gauge_{strain_gauges_id}.csv", "wb",
                                         encoding="ASCII"), lineterminator='\n')
            self.headline_1.append(self.initial_weight)
            csv_output.writerow(self.headline_1)  # write headline row with weight values
            measurement = list(map(int, measurement))
            self.temp_1.append(round(numpy.average(measurement)))
            csv_output.writerow(self.temp_1)  # write measurements


if __name__ == '__main__':
    rospy.init_node("Calib_strain_gauges", anonymous=True)
    calib = CalibGauges()
    calib.run_test()

# Output of the file Motor_11_strain_gauge_0.csv
# 100,200,300,400,500
# 0.0,-1.0,1.0,0.0,1.0
