#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PolygonStamped, Point32


class Visualise:

    def __init__(self, ref_frame):
        self.polygon_msg = PolygonStamped()
        self.polygon_msg.header.frame_id = "/rh_forearm"  # CHANGE HERE: odom/map
        self.polygon_msg.header.frame_id = ref_frame

        self.pub = rospy.Publisher('/grasp_quality_measure', PolygonStamped, queue_size=1)

    def new_polygon_point(self, (x, y, z), points_list):
        point = Point32()
        point.x, point.y, point.z = x, y, z
        points_list.append(point)

    def publish_obstacle_msg(self, trans):
        # Adjust the topic for navigation or remap
        self.polygon_msg.header.stamp = rospy.Time.now()
        self.polygon_msg.polygon.points = []

        self.new_polygon_point(trans['rh_fftip'], self.polygon_msg.polygon.points)
        self.new_polygon_point(trans['rh_mftip'], self.polygon_msg.polygon.points)
        self.new_polygon_point(trans['rh_rftip'], self.polygon_msg.polygon.points)
        self.new_polygon_point(trans['rh_lftip'], self.polygon_msg.polygon.points)
        self.new_polygon_point(trans['rh_thtip'], self.polygon_msg.polygon.points)

        self.pub.publish(self.polygon_msg)
