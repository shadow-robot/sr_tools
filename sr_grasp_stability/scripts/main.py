#!/usr/bin/env python

import rospy
from sr_grasp_stability.tf2_computation import TfComputator
from sr_grasp_stability.grasp_computation import poly_3d_area, get_centre_of_grasp, measure_grasp_polygon_angles
from sr_grasp_stability.polygon_visual import PolygonVisual
from sr_grasp_stability.marker_visual import MarkerVisual

rospy.init_node('sr_grasp_stability')

# Set reference frame
ref_frame = 'world'

TF_computator = TfComputator()
Visual = PolygonVisual(ref_frame)
marker = MarkerVisual(ref_frame)

fingertips = {}

while not rospy.is_shutdown():

    rate = rospy.Rate(10.0)

    # State which fingertips are used in grasp calculation
    list_of_fingertips = ['rh_fftip', 'rh_mftip', 'rh_rftip', 'rh_lftip', 'rh_thtip']  # Hand E fingertip frames:

    # Get fingertip positions relative to reference frame
    fingertips = TF_computator.get_fingertips(list_of_fingertips, ref_frame)

    centre_of_grasp = get_centre_of_grasp(fingertips)

    poly_area = poly_3d_area(fingertips)

    measure_using_angles = measure_grasp_polygon_angles(fingertips)

    Visual.publish_obstacle_msg(fingertips)
    marker.publish_marker(centre_of_grasp)

    rospy.loginfo("Grasp: Area = %s, Shape = %s", poly_area, measure_using_angles)

    rate.sleep()
