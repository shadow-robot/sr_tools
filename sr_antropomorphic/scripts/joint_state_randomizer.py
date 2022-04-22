#!/usr/bin/env python3

import sys
import random

import rospy
import sensor_msgs.msg

from joint_state_publisher import JointStatePublisher
from joint_state_publisher import get_param


class JointStateRandomiser(JointStatePublisher):

    def __init__(self, *args, **kwargs):
        super(JointStateRandomiser, self).__init__(*args, **kwargs)

    def loop(self):
        hz = get_param("rate", 10)  # 10hz
        total_samples = get_param("samples", 100)
        r = rospy.Rate(hz)

        delta = get_param("delta", 0.0)

        # Publish Joint States
        samples = 0
        while not rospy.is_shutdown() and samples < total_samples:
            msg = sensor_msgs.msg.JointState()
            msg.header.stamp = rospy.Time.now()

            if delta > 0:
                self.update(delta)

            # Initialize msg.position, msg.velocity, and msg.effort.
            has_position = len(self.dependent_joints.items()) > 0
            has_velocity = False
            has_effort = False
            for name, joint in self.free_joints.items():
                if not has_position and 'position' in joint:
                    has_position = True
                if not has_velocity and 'velocity' in joint:
                    has_velocity = True
                if not has_effort and 'effort' in joint:
                    has_effort = True
            num_joints = (len(self.free_joints.items()) +
                          len(self.dependent_joints.items()))
            if has_position:
                msg.position = num_joints * [0.0]
            if has_velocity:
                msg.velocity = num_joints * [0.0]
            if has_effort:
                msg.effort = num_joints * [0.0]

            for i, name in enumerate(self.joint_list):
                
                msg.name.append(str(name))
                joint = None

                # Add Free Joint
                if name in self.free_joints:
                    joint = self.free_joints[name]
                    factor = 1
                    offset = 0
                # Add Dependent Joint
                elif name in self.dependent_joints:
                    param = self.dependent_joints[name]
                    parent = param['parent']
                    factor = param.get('factor', 1)
                    offset = param.get('offset', 0)
                    # Handle recursive mimic chain
                    recursive_mimic_chain_joints = [name]
                    while parent in self.dependent_joints:
                        if parent in recursive_mimic_chain_joints:
                            error_message = "Found an infinite recursive mimic chain"
                            rospy.logerr("%s: [%s, %s]", error_message, ', '.join(recursive_mimic_chain_joints), parent)
                            sys.exit(1)
                        recursive_mimic_chain_joints.append(parent)
                        param = self.dependent_joints[parent]
                        parent = param['parent']
                        offset += factor * param.get('offset', 0)
                        factor *= param.get('factor', 1)
                    joint = self.free_joints[parent]

                if has_position and 'position' in joint:
                    if "rh_WRJ" in name:
                        continue
                    msg.position[i] = random.uniform(joint['min'], joint['max'])
                if has_velocity and 'velocity' in joint:
                    msg.velocity[i] = joint['velocity'] * factor
                if has_effort and 'effort' in joint:
                    msg.effort[i] = joint['effort']

            if msg.name or msg.position or msg.velocity or msg.effort:
                # Only publish non-empty messages
                self.pub.publish(msg)
            try:
                r.sleep()
            except rospy.exceptions.ROSTimeMovedBackwardsException:
                pass
        
            samples = samples + 1
        

if __name__ == '__main__':
    try:
        rospy.init_node('joint_state_publisher')
        jsp = JointStateRandomiser()
        jsp.loop()

    except rospy.ROSInterruptException:
        pass
