#!/usr/bin/env python

import rospy
#import roslib; roslib.load_manifest('visualization_marker_tutorials')
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import Point


class Visualize:

    def __init__(self):
        # rospy.init_node('polygon_visualization')
        topic = 'visualization_marker_array'
        topic_2 = 'visualization_marker'

        self.publisher = rospy.Publisher(topic, MarkerArray, queue_size=100)
        self.publisher_g = rospy.Publisher(topic_2, Marker, queue_size=100)

        self.finger_tips_array = MarkerArray()
        self.polygon = Marker()

        self.polygon.type = self.polygon.LINE_STRIP
        self.polygon.header.frame_id = "/rh_forearm"
        self.polygon.header.stamp = rospy.Time(0)
        self.polygon.ns = "grasp"
        self.polygon.action = self.polygon.ADD
        self.polygon.pose.orientation.w = 1.0
        self.polygon.id = 10
        self.polygon.scale.x = 1
        self.polygon.color.a = 1.0
        self.polygon.color.r = 0.0
        self.polygon.color.g = 1.0
        self.polygon.color.b = 0.0

        self.finger_tip = Marker()
        self.finger_tip.header.stamp = rospy.Time(0)
        # self.finger_tip.header.frame_id = "/rh_forearm"
        # self.finger_tip.ns = "grasp"

        self.p = Point()

    def vis(self, transform):
        self.finger_tips_array.markers = []
        # self.polygon.markers = []
        for i in range(len(transform)):
            self.finger_tips_array.markers.append(0)
            # self.polygon.markers.append(0)

        finger = 0
        for finger_pos in transform:
            self.finger_tip.id = finger
            self.finger_tip.type = self.finger_tip.SPHERE
            self.finger_tip.action = self.finger_tip.ADD
            # finger_tip.duration = rospy.Duration()
            self.finger_tip.pose.orientation.x = 0.0
            self.finger_tip.pose.orientation.y = 0.0
            self.finger_tip.pose.orientation.z = 0.0
            self.finger_tip.pose.orientation.w = 1.0
            self.finger_tip.color.a = 1.0
            self.finger_tip.color.r = 1.0
            self.finger_tip.color.g = 0.6
            self.finger_tip.color.b = 0.1

            self.finger_tip.scale.x = 0.1
            self.finger_tip.scale.y = 0.1
            self.finger_tip.scale.z = 0.1

            self.finger_tip.pose.position.x = transform[finger_pos][0]
            self.finger_tip.pose.position.y = transform[finger_pos][1]
            self.finger_tip.pose.position.z = transform[finger_pos][2]

            self.finger_tips_array.markers.append(self.finger_tip)

            self.p.x = transform[finger_pos][0]
            self.p.y = transform[finger_pos][1]
            self.p.z = transform[finger_pos][2]

            self.polygon.points.append(self.p)

            finger += 1

        # self.publisher.publish(self.finger_tips_array)
        self.publisher_g.publish(self.polygon)
