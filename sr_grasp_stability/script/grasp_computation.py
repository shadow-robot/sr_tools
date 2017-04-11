#!/usr/bin/env python

import rospy
from sr_grasp_stability.tf2_computation import TfComputator
# from src.polygon_computation import Polygon
# from visualization_2 import Visualize

from std_msgs.msg import Header
from geometry_msgs.msg import Polygon, Point32, PolygonStamped

pub = rospy.Publisher('grasp_quality_measure', Polygon, queue_size=10)

rospy.init_node('sr_grasp_stability')

TF_comp = TfComputator()  # Create an instance
Poly = Polygon()  # Create an instance
# Visual = Visualize()

trans = {}

point = Point32()

while not rospy.is_shutdown():

    rate = rospy.Rate(10.0)

    trans, exc = TF_comp.get_finger_tips()

    if not exc:
        print trans['rh_fftip']
        rospy.loginfo("The grasp 2d-polygon area is %s", Poly.measure_grasp_polygon_area(trans))
        # Visual.vis(trans)

    poly_points = []

    if 'rh_rfdistal' in trans:
        for key, value in trans.items():
            point.x, point.y, point.z = value
            poly_points.append(point)

            # print key, value

    # print poly_points

    polygon = Polygon(poly_points)

    poly_stamped = PolygonStamped(polygon)

    pub.publish(poly_stamped)

    print poly_stamped

    rate.sleep()
