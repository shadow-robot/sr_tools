#!/usr/bin/env python

import rospy
from sr_grasp_stability.tf2_computation import TfComputator
from sr_grasp_stability.polygon_computation import measure_grasp_polygon_area
from sr_grasp_stability.visualization import Visualise


rospy.init_node('sr_grasp_stability')

# Set reference frame
ref_frame = 'world'

TF_comp = TfComputator()  # Create an instance
# Poly = Polygon()  # Create an instance
Visual = Visualise(ref_frame)

trans = {}

while not rospy.is_shutdown():

    rate = rospy.Rate(10.0)

    # Get finger-tips position relative to reference frame
    trans, exc = TF_comp.get_finger_tips(ref_frame)

    if not exc:

        poly_area = measure_grasp_polygon_area(trans)
        Visual.publish_obstacle_msg(trans)

        rospy.loginfo("Grasp polygon area = %s", poly_area)


    rate.sleep()
