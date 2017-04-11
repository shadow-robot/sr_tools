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

        self.publisher = rospy.Publisher(topic, MarkerArray, queue_size=1000)
        self.publisher_g = rospy.Publisher(topic_2, Marker, queue_size=100)

        self.finger_tips = Marker()
        self.polygon = Marker()

        self.finger_tips_array = MarkerArray()
        self.polygon_array = MarkerArray()

        # self.MARKERS_MAX = 100

        self.finger_tips.header.frame_id = "/rh_forearm"
        self.polygon.header.frame_id = "/grasp"

        self.finger_tips.type = self.finger_tips.SPHERE
        self.polygon.type = self.polygon.LINE_STRIP

        self.finger_tips.action = self.finger_tips.ADD
        self.polygon.action = self.polygon.ADD

        self.finger_tips.id = 0
        self.polygon.id = 10

        self.finger_tips.pose.orientation.w = 1.0
        self.polygon.pose.orientation.w = 1.0

        self.finger_tips.scale.x = 0.01
        self.finger_tips.scale.y = 0.01
        self.finger_tips.scale.z = 0.01
        self.finger_tips.color.a = 1.0
        self.finger_tips.color.r = 0.0
        self.finger_tips.color.g = 0.0
        self.finger_tips.color.b = 1.0

        self.polygon.scale.x = 0.1
        # self.polygon.scale.y = 0.2
        # self.polygon.scale.z = 0.2
        self.polygon.color.a = 1.0
        self.polygon.color.r = 0.0
        self.polygon.color.g = 1.0
        self.polygon.color.b = 0.0

    def vis(self, transform):
        self.finger_tips_array.markers = []
        self.polygon_array.markers = []
        for i in range(len(transform)):
            self.finger_tips_array.markers.append(0)
            self.polygon_array.markers.append(0)
        self.polygon_array.markers.append(0)
        p = Point()

        print self.finger_tips_array.markers[0]
        print self.finger_tips_array.markers[2]

        finger = 0
        for finger_pos in transform:
            '''
            self.finger_tips.pose.position.x = transform[finger_pos][0]
            self.finger_tips.pose.position.y = transform[finger_pos][1]
            self.finger_tips.pose.position.z = transform[finger_pos][2]
            '''

            p.x = transform[finger_pos][0]
            p.y = transform[finger_pos][1]
            p.z = transform[finger_pos][2]

            # self.finger_tips_array.markers.append(self.finger_tips.pose)
            print finger_pos
            # print transform[finger_pos]
            print self.finger_tips.pose.position.x

            self.polygon.pose.position.x = transform[finger_pos][0]
            self.polygon.pose.position.y = transform[finger_pos][1]
            self.polygon.pose.position.z = transform[finger_pos][2]

            self.finger_tips_array.markers[finger] = self.finger_tips
            self.polygon_array.markers[finger] = self.polygon
            if finger == 0:
                self.polygon_array.markers[len(transform)-1] = self.polygon

            finger = finger + 1

        #print self.finger_tips_array.markers[0]
        #print self.finger_tips_array.markers[2]

        '''
        # Renumber the marker IDs
        id = 0
        for m in self.markerArray.markers:
            m.id = id
            id += 1
        '''

        # Publish the MarkerArray
        self.publisher.publish(self.finger_tips_array)
        # self.publisher_g.publish(self.polygon_array)
