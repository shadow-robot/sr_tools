#!/usr/bin/env python

import rospy

from geometry_msgs.msg import PolygonStamped, Point32

def talker():
    pub = rospy.Publisher('grasp_quality_measure', Point32, queue_size=10)
    rospy.init_node('grasp_measurement_talker', anonymous=True)
    rate = rospy.Rate(10)  # 10hz

    grasp_metric = 0.1

    while not rospy.is_shutdown():

        # hello_str = "hello world %s" % rospy.get_time()

        rospy.loginfo(grasp_metric)

        pub.publish(grasp_metric)
        rate.sleep()

        grasp_metric += 0.1

        if grasp_metric > 10:
            grasp_metric = 0.1

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass



