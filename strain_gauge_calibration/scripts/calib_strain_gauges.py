#!/usr/bin/env python

import os
import rospy
import numpy
import csv
from diagnostic_msgs.msg import DiagnosticArray


class Calib_gauges():
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

    def data_callback(self, data):

        # iterate over the DiagnosticStatus message
        for i, value in enumerate(data.status):

            fields = {}

            # iterate over values array and store them in a dictionary with index and relative value
            for key, s in enumerate(value.values):

                fields[i, s.key] = s.value

            # iterate over dictionary elements to search the wanted motor_id and the relative strain_gauge value
            for index, motor_id in fields.items():

                if fields.get((index[0], 'Motor ID')) == "%s" % self.motor_id:
                    if self.strain_gauges_id == 0:
                        # store measurement in array
                        self.measurement.append(fields.get((index[0], 'Strain Gauge Left')))
                    else:
                        # store measurement in array
                        self.measurement.append(fields.get((index[0], 'Strain Gauge Right')))

    def run_test(self):

        self.motor_id = raw_input("insert motor id: ")

        for self.strain_gauges_id in range(0, 2):

            self.initial_weight = 0

            for weight in range(0, self.number_of_tests):

                self.initial_weight += self.incremental_weight

                print("Apply %s g to strain_gauges %s" % (self.initial_weight, self.strain_gauges_id))
                raw_input("Press enter when you are done...")

                # Subscribe diagnostic topic to collect data
                self.sub = rospy.Subscriber("/diagnostics_agg", DiagnosticArray, self.data_callback)
                print("Measuring...")

                rospy.sleep(5.0)  # give time to collect measurements
                self.sub.unregister()  # stop subscribing topic

                self.update_csv_file(self.measurement, self.strain_gauges_id)  # write measurements into csv file
                self.measurement = []  # empty measurement array

    def update_csv_file(self, measurement, strain_gauges_id):

        if strain_gauges_id == 0:
            # create csv to write
            csv_output = csv.writer(open("Motor_%s_strain_gauge_%s.csv" % (self.motor_id, strain_gauges_id), "wb"),
                                    lineterminator='\n')
            self.headline_0.append(self.initial_weight)
            csv_output.writerow(self.headline_0)   # write headline row with weight values
            measurement = map(int, measurement)  # cast values to int
            self.temp_0.append(round(numpy.average(measurement)))  # compute values average
            csv_output.writerow(self.temp_0)  # write measurements

        elif strain_gauges_id == 1:
            # create csv to write
            csv_output = csv.writer(open("Motor_%s_strain_gauge_%s.csv" % (self.motor_id, strain_gauges_id), "wb"),
                                    lineterminator='\n')
            self.headline_1.append(self.initial_weight)
            csv_output.writerow(self.headline_1)  # write headline row with weight values
            measurement = map(int, measurement)
            self.temp_1.append(round(numpy.average(measurement)))
            csv_output.writerow(self.temp_1)  # write measurements


if __name__ == '__main__':

    rospy.init_node("Calib_strain_gauges", anonymous=True)
    calib = Calib_gauges()
    calib.run_test()

# Output of the file Motor_11_strain_gauge_0.csv
# 100,200,300,400,500
# 0.0,-1.0,1.0,0.0,1.0
