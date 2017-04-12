#!/usr/bin/env python

import rospy

from std_msgs.msg import String
# from geometry_msgs.msg import PointStamped
from visualization_msgs.msg import Marker


class PlaceMarker:

    def __init__(self, ref_frame):
        # self.pub = rospy.Publisher('state', PointStamped, queue_size=10)

        self.markerPub = rospy.Publisher('grasp_centre_Marker', Marker, queue_size=10)

        # rospy.Subscriber("action", String, self.move_callback)

        self.robotMarker = Marker()
        self.robotMarker.header.frame_id = ref_frame

        self.robotMarker.pose.orientation.x = 0
        self.robotMarker.pose.orientation.y = 0
        self.robotMarker.pose.orientation.z = 0
        self.robotMarker.pose.orientation.w = 1.0

        self.robotMarker.type = 2  # Sphere
        self.robotMarker.scale.x = 0.005
        self.robotMarker.scale.y = 0.005
        self.robotMarker.scale.z = 0.005
        self.robotMarker.color.r = 0.0
        self.robotMarker.color.g = 1.0
        self.robotMarker.color.b = 0.0
        self.robotMarker.color.a = 1.0

    def set_marker_position(self, (x, y, z)):
        self.robotMarker.pose.position.x = x
        self.robotMarker.pose.position.y = y
        self.robotMarker.pose.position.z = z

    def publish_marker(self, point):
        self.robotMarker.header.stamp = rospy.get_rostime()
        self.set_marker_position(point)
        self.markerPub.publish(self.robotMarker)
