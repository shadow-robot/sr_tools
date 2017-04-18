#!/usr/bin/env python

import rospy
from sr_grasp_stability.tf2_computation import TfComputator
from sr_grasp_stability.polygon_computation import poly_3d_area, get_centre_of_grasp, measure_grasp_polygon_angles
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

    # State which fingertips are used in grasp calculation
    list_of_fingertips = ['rh_fftip', 'rh_mftip', 'rh_rftip', 'rh_lftip', 'rh_thtip']  # Hand E fingertip frames:

    # Get fingertip positions relative to reference frame
    fingertips, exc = TF_comp.get_fingertips(list_of_fingertips, ref_frame)

    if not exc:

        measure_using_angles = measure_grasp_polygon_angles(fingertips)
        print measure_using_angles

    #  poly_area = poly_3d_area(fingertips)
        centre_of_grasp = get_centre_of_grasp(fingertips)

        Visual.publish_obstacle_msg(fingertips)
        marker.publish_marker(centre_of_grasp)
    #
    #     rospy.loginfo("Grasp polygon area = %s", poly_area)

    rate.sleep()
