#!/usr/bin/env python

import rospy
from sr_grasp_stability.tf2_computation import TfComputator
from sr_grasp_stability.polygon_computation import poly_3d_area, get_centre_of_grasp
from sr_grasp_stability.visualization import Visualise
from sr_grasp_stability.place_marker import PlaceMarker

rospy.init_node('sr_grasp_stability')

# Set reference frame
ref_frame = 'world'

TF_comp = TfComputator()
Visual = Visualise(ref_frame)
marker = PlaceMarker(ref_frame)

fingertips = {}

while not rospy.is_shutdown():

    rate = rospy.Rate(10.0)

    # Get finger-tips position relative to reference frame
    fingertips, exc = TF_comp.get_fingertips(ref_frame)

    if not exc:

        poly_area = poly_3d_area(fingertips)
        centre_of_grasp = get_centre_of_grasp(fingertips)

        Visual.publish_obstacle_msg(fingertips)
        marker.publish_marker(centre_of_grasp)

        rospy.loginfo("Grasp polygon area = %s", poly_area)

    rate.sleep()
