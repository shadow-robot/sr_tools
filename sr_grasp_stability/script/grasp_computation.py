#!/usr/bin/env python

import rospy
from sr_grasp_stability.tf2_computation import TfComputator
# from src.polygon_computation import Polygon
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

while not rospy.is_shutdown():

    rate = rospy.Rate(10.0)

    # Get finger-tips position relative to reference frame
    trans, exc = TF_comp.get_finger_tips(ref_frame)

    # Create polygon message, populate it and publish for rViz
    polygon_msg.polygon.points = []
    if not exc:
        for key, value in trans.items():
            point = Point32()
            point.x, point.y, point.z = value
            polygon_msg.polygon.points.append(point)

        pub.publish(polygon_msg)

    rate.sleep()
