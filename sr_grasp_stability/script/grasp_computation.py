#!/usr/bin/env python

import rospy
from tf_computation import TfComputator
from polygon_computation import Polygon

rospy.init_node('sr_grasp_stability')

TF_comp = TfComputator()  # Create an instance
Poly = Polygon()  # Create an instance

trans = {}
rot = {}

while not rospy.is_shutdown():

    rate = rospy.Rate(10.0)

    trans, rot = TF_comp.get_finger_tips()
    rospy.loginfo("The grasp 2d-polygon area is %s", Poly.measure_grasp_polygon_area(trans))
    # print trans

    rate.sleep()
