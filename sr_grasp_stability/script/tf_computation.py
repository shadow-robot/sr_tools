#!/usr/bin/env python

# import roslib
# roslib.load_manifest('learning_tf')
import rospy
import tf

if __name__ == '__main__':

    rospy.init_node('sr_grasp_stability')

    listener = tf.TransformListener()

    # rospy.wait_for_service('spawn')
    # spawner = rospy.ServiceProxy('spawn', turtlesim.srv.Spawn)
    # spawner(4, 2, 0, 'turtle2')

    # grasp_stability = rospy.Publisher('grasp_stability', geometry_msgs.msg.Twist, queue_size=1)

    rate = rospy.Rate(10.0)  # Hz

    trans = {}

    while not rospy.is_shutdown():

        # figure tip frames:
        # '/rh_rftip', '/rh_lftip', '/rh_mftip', '/rh_thtip', '/rh_fftip',

        try:
            (trans['rh_ffdistal'], rot_rh_rftip) = listener.lookupTransform('/rh_ffdistal', '/rh_forearm', rospy.Time(0))
            (trans['rh_ffdistal'], rot_rh_rftip) = listener.lookupTransform('/rh_lfdistal', '/rh_forearm', rospy.Time(0))
            (trans['rh_mfdistal'], rot_rh_rftip) = listener.lookupTransform('/rh_mfdistal', '/rh_forearm', rospy.Time(0))
            (trans['rh_ffdistal'], rot_rh_lftip) = listener.lookupTransform('/rh_rfdistal', '/rh_forearm', rospy.Time(0))

            (trans['rh_thdistal'], rot_rh_thtip) = listener.lookupTransform('/rh_thdistal', '/rh_forearm', rospy.Time(0))

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        # rospy.loginfo("transform_1 %s", trans['rh_ffdistal'])

        print trans

        # angular = 4 * math.atan2(trans[1], trans[0])
        # linear = 0.5 * math.sqrt(trans[0] ** 2 + trans[1] ** 2)
        # cmd = geometry_msgs.msg.Twist()
        # cmd.linear.x = linear
        # cmd.angular.z = angular
        # turtle_vel.publish(cmd)

        rate.sleep()


# class TfComputator():
#
#     def __innit__(self):
#
#
# if __name__ == '__main__':
#     rospy.init_node("mouse_tele_op", anonymous=True)
#     tf_computator = TfComputator()
#     # update pose
#     MouseSim.update_robot_pose()
#     rospy.spin()