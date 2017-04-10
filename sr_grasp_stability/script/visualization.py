#!/usr/bin/env python

import rospy
import roslib; roslib.load_manifest('visualization_marker_tutorials')
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
import math


class Visualize:

    def __init__(self):
        topic = 'visualization_marker_array'
        self.publisher = rospy.Publisher(topic, MarkerArray)

        rospy.init_node('polygon_visualization')

        self.markerArray = MarkerArray()

        self.count = 0
        self.MARKERS_MAX = 100

    def vis(self, ):
        marker = Marker()
        marker.header.frame_id = "/hand"
        marker.type = marker.SPHERE
        marker.action = marker.ADD
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.pose.orientation.w = 1.0
        marker.pose.position.x = math.cos(self.count / 50.0)
        marker.pose.position.y = math.cos(self.count / 40.0)
        marker.pose.position.z = math.cos(self.count / 30.0)

        # We add the new marker to the MarkerArray, removing the oldest
        # marker from it when necessary
        if(self.count > self.MARKERS_MAX):
            self.markerArray.markers.pop(0)

            self.markerArray.markers.append(marker)

        # Renumber the marker IDs
        id = 0
        for m in self.markerArray.markers:
            m.id = id
            id += 1

        # Publish the MarkerArray
        self.publisher.publish(self.markerArray)

        self.count += 1