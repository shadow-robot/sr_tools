#!/usr/bin/env python

import rospy
from sr_grasp_stability.tf2_computation import TfComputator
from sr_grasp_stability.polygon_computation import measure_grasp_polygon_area
# from visualization_2 import Visualize

from std_msgs.msg import Header
from geometry_msgs.msg import Point32, PolygonStamped

pub = rospy.Publisher('grasp_quality_measure', PolygonStamped, queue_size=10)

rospy.init_node('sr_grasp_stability')

TF_comp = TfComputator()  # Create an instance
# Poly = Polygon()  # Create an instance
# Visual = Visualize()

trans = {}
# Set reference frame
ref_frame = 'world'

polygon_msg = PolygonStamped()
polygon_msg.header.frame_id = ref_frame

def new_polygon_point((x, y, z), points_list):
    point = Point32()
    point.x, point.y, point.z = x, y, z
    points_list.append(point)

while not rospy.is_shutdown():

    rate = rospy.Rate(10.0)

    # Get finger-tips position relative to reference frame
    trans, exc = TF_comp.get_finger_tips(ref_frame)

    # Create polygon message, populate it and publish for rViz
    polygon_msg.polygon.points = []
    if not exc:

        poly_area = measure_grasp_polygon_area(trans)

        # for key, value in trans.items():
        #     point = Point32()
        #     point.x, point.y, point.z = value
        #     polygon_msg.polygon.points.append(point)
        #     print key, value

        new_polygon_point(trans['rh_fftip'], polygon_msg.polygon.points)
        new_polygon_point(trans['rh_mftip'], polygon_msg.polygon.points)
        new_polygon_point(trans['rh_rftip'], polygon_msg.polygon.points)
        new_polygon_point(trans['rh_lftip'], polygon_msg.polygon.points)
        new_polygon_point(trans['rh_thtip'], polygon_msg.polygon.points)

        # rh_fftip rh_mftip rh_rftip rh_lftip rh_thtip


        pub.publish(polygon_msg)
        rospy.loginfo("Grasp polygon area = %s", poly_area)


    rate.sleep()
