#!/usr/bin/env python

import rospy
import tf

from tf_computation import tf_comp

tf_comp = tf_comp()

state = {}
while not rospy.is_shutdown():

    rate = rospy.Rate(10.0)

    state = tf_comp.listening()
    print state

    rate.sleep()
