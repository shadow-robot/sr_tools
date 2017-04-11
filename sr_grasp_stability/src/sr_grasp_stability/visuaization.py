#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PolygonStamped, Point32


class Visualise:

    def __init__(self):
        rospy.init_node("publish_obstacles")
        self.polygon_msg.header.frame_id = "/rh_forearm"  # CHANGE HERE: odom/map
        self.pub = rospy.Publisher('/grasp_quality_measure', PolygonStamped, queue_size=1)
        # Create empty obstacle message that will be filled in afterwards
        self.polygon_msg = PolygonStamped()

    def publish_obstacle_msg(self, trans):
        # Adjust the topic for navigation or remap

        self.polygon_msg.header.stamp = rospy.Time.now()
        self.polygon_msg.polygon.points = []

        for key, value in trans.items():
            point = Point32()
            point.x, point.y, point.z = value
            self.polygon_msg.polygon.points.append(point)

        self.pub.publish(self.polygon_msg)
