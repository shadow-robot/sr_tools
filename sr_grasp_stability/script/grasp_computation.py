#!/usr/bin/env python

import rospy
from tf2_computation import TfComputator
from polygon_computation import Polygon
# from visualization_2 import Visualize

rospy.init_node('sr_grasp_stability')

TF_comp = TfComputator()  # Create an instance
Poly = Polygon()  # Create an instance
# Visual = Visualize()

trans = {}
rot = {}

while not rospy.is_shutdown():

    rate = rospy.Rate(10.0)

    trans, exc = TF_comp.get_finger_tips()

    if not exc:
        print trans['rh_fftip']
        rospy.loginfo("The grasp 2d-polygon area is %s", Poly.measure_grasp_polygon_area(trans))
        # Visual.vis(trans)

    rate.sleep()
