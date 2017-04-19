#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PolygonStamped, Point32


class PolygonVisual:
    """
    Makes a polygon for RViz and publishes it
    """
    def __init__(self, ref_frame):
        self.polygon_msg = PolygonStamped()
        self.polygon_msg.header.frame_id = ref_frame

        self.pub = rospy.Publisher('/grasp_quality_measure', PolygonStamped, queue_size=1)

    def new_polygon_point(self, translation, points_list):
        point = Point32()
        point.x, point.y, point.z = translation.x, translation.y, translation.z
        points_list.append(point)

    def publish_obstacle_msg(self, fingertips):
        self.polygon_msg.header.stamp = rospy.Time.now()
        self.polygon_msg.polygon.points = []

        for fingertip in fingertips:
            self.new_polygon_point(fingertip['transform'].translation, self.polygon_msg.polygon.points)

        self.pub.publish(self.polygon_msg)
