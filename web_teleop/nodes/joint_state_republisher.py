#!/usr/bin/env python

from asyncore import read

from requests import head
import rospy
from std_msgs.msg import Float64, Float64MultiArray
from joint_state_reader import JointStateReader


def wait_for_time():
    """Wait for simulated time to begin.
    """
    while rospy.Time().now().to_sec() == 0:
        pass


def main():
    rospy.init_node('joint_state_republisher')
    wait_for_time()
    joints = ['torso_lift_joint', 'head_pan_joint', 'head_tilt_joint', 'shoulder_pan_joint', 'shoulder_lift_joint',
              'upperarm_roll_joint', 'elbow_flex_joint', 'forearm_roll_joint', 'wrist_flex_joint', 'wrist_roll_joint']

    prefix = "joint_state_republisher/"

    torso_pub = rospy.Publisher(prefix + "torso_lift_joint", Float64)
    head_pub = rospy.Publisher(prefix + "head", Float64MultiArray)
    arm_pub = rospy.Publisher(prefix + "arm", Float64MultiArray)

    reader = JointStateReader()
    rospy.sleep(0.5)

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        # TODO: get torso joint value
        # TODO: publish torso joint value
        joint_values = reader.get_joints(joints)
        # head_pan_value = reader.get_joint('head_pan_joint')
        # head_tilt_value = reader.get_joint('head_tilt_joint')
        torso_pub.publish(joint_values[0])

        head_data = Float64MultiArray()
        head_data.data = joint_values[1:3]
        head_pub.publish(head_data)

        arm_data = Float64MultiArray()
        arm_data.data = joint_values[3:]
        arm_pub.publish(arm_data)
        
        rate.sleep()


if __name__ == '__main__':
    main()