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

    while not rospy.is_shutdown():

        # figure tip frames:
        # '/rh_rftip', '/rh_lftip', '/rh_mftip', '/rh_thtip', '/rh_fftip',

        try:
            (trans_rh_rftip, rot_rh_rftip) = listener.lookupTransform('rh_rftip', 'rh_forearm', rospy.Time(0))

        # rh_ffknuckle

        # (trans_rh_rftip, rot_rh_rftip) = listener.lookupTransform('/rh_ffdistal', '/rh_rftip', rospy.Time(0))
        # (trans_rh_lftip, rot_rh_lftip) = listener.lookupTransform('/world', '/rh_lftip', rospy.Time(0))
        # (trans_rh_mftip, rot_rh_mftip) = listener.lookupTransform('/world', '/rh_mftip', rospy.Time(0))
        # (trans_rh_thtip, rot_rh_thtip) = listener.lookupTransform('/world', '/rh_thtip', rospy.Time(0))
        # (trans_rh_fftip, rot_rh_fftip) = listener.lookupTransform('/world', '/rh_fftip', rospy.Time(0))

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        # rospy.loginfo("transform_1 %s", trans_rh_rftip[0])

        rospy.loginfo("transform_1 %s", trans_rh_rftip)


        # print trans_rh_rftip[0], trans_rh_rftip[1], trans_rh_rftip[2]

        # angular = 4 * math.atan2(trans[1], trans[0])
        # linear = 0.5 * math.sqrt(trans[0] ** 2 + trans[1] ** 2)
        # cmd = geometry_msgs.msg.Twist()
        # cmd.linear.x = linear
        # cmd.angular.z = angular
        # turtle_vel.publish(cmd)

        rate.sleep()