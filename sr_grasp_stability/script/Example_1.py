#!/usr/bin/env python
import rospy, math
# from teb_local_planner.msg import ObstacleMsg

from base.src.sr_tools.sr_grasp_stability.src.sr_grasp_stability.tf_computation import TfComputator


from geometry_msgs.msg import PolygonStamped, Point32


def publish_obstacle_msg():
    # Adjust the topic for navigation or remap
    pub = rospy.Publisher('/grasp_quality_measure', PolygonStamped, queue_size=1)

    rospy.init_node("publish_obstacles")

    r = rospy.Rate(10)  # 10hz

    TF_comp = TfComputator()  # Create an instance

    # Create empty obstacle message that will be filled in afterwards
    polygon_msg = PolygonStamped()
    polygon_msg.header.frame_id = "/rh_forearm"  # CHANGE HERE: odom/map

    while not rospy.is_shutdown():

        polygon_msg.header.stamp = rospy.Time.now()

        # # Add point polygon
        # polygon_msg.polygon.points = [Point32(), Point32(), Point32()]
        # polygon_msg.polygon.points[0].x = 1
        # polygon_msg.polygon.points[0].y = 0
        # polygon_msg.polygon.points[0].z = 0
        # polygon_msg.polygon.points[1].x = 1
        # polygon_msg.polygon.points[1].y = 1
        # polygon_msg.polygon.points[1].z = 0
        # polygon_msg.polygon.points[2].x = 1
        # polygon_msg.polygon.points[2].y = 1
        # polygon_msg.polygon.points[2].z = 1

        trans, rot = TF_comp.get_finger_tips()
        polygon_msg.polygon.points = []

        if 'rh_rfdistal' in trans:
            for key, value in trans.items():
                point = Point32()
                point.x, point.y, point.z = value
                polygon_msg.polygon.points.append(point)


        # # Add line obstacle
        # obstacle_msg.obstacles.append(PolygonStamped())
        # line_start = Point32()
        # line_start.x = -2.5
        # line_start.y = 0.5
        # line_end = Point32()
        # line_end.x = -2.5
        # line_end.y = 2
        # obstacle_msg.obstacles[1].polygon.points = [line_start, line_end]
        #
        # # Add polygon obstacle
        # obstacle_msg.obstacles.append(PolygonStamped())
        # v1 = Point32()
        # v1.x = -1
        # v1.y = -1
        # v2 = Point32()
        # v2.x = -0.5
        # v2.y = -1.5
        # v3 = Point32()
        # v3.x = 0
        # v3.y = -1
        # obstacle_msg.obstacles[2].polygon.points = [v1, v2, v3]

        # Publish the message
        pub.publish(polygon_msg)
        # print polygon_msg

        r.sleep()


if __name__ == '__main__':
    try:
        publish_obstacle_msg()
    except rospy.ROSInterruptException:
        pass