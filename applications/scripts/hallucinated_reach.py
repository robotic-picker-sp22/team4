#! /usr/bin/env python

import copy
from geometry_msgs.msg import PoseStamped
from ar_track_alvar_msgs.msg import AlvarMarkers
from joint_state_reader import JointStateReader
import robot_api
import rospy


def wait_for_time():
    """Wait for simulated time to begin.
    """
    while rospy.Time().now().to_sec() == 0:
        pass


class ArTagReader(object):
    def __init__(self):
        self.markers = []

    def callback(self, msg):
        self.markers = msg.markers


def main():
    rospy.init_node('hallucination_search')
    wait_for_time()
    joint_listener = JointStateReader()


    start = PoseStamped()
    start.header.frame_id = 'base_link'
    start.pose.position.x = 0.5
    start.pose.position.y = 0.5
    start.pose.position.z = 0.75
    arm = robot_api.Arm()
    arm.move_to_pose(start)
    rospy.loginfo('Moved to marker {}'.format(start.header.frame_id))
                                                                               
    reader = ArTagReader()
    sub = rospy.Subscriber('/ar_pose_marker', AlvarMarkers, reader.callback) # Subscribe to AR tag poses, use reader.callback
    
    while len(reader.markers) == 0:
        rospy.sleep(0.1)
    
    names = robot_api.ArmJoints.names()
    joints = joint_listener.get_joints(names)
    for marker in reader.markers:
        # TODO: get the pose to move to
        marker.pose.header = marker.header
        print(marker.header)
        pose = copy.deepcopy(marker.pose)
        pose.pose.position.x -= 0.1
        pose.pose.orientation = copy.deepcopy(start.pose.orientation)
        joints = arm.compute_ik(pose, joint_name=names, joint_pos=joints, nums=True)
        if joints != False:
            result = arm.move_to_joint(names, joints, replan=True)
        else:
            result = None
            joints = joint_listener.get_joints(names)
        if result == "SUCCESS":
            rospy.loginfo('Moved to marker {}'.format(marker.id))
            # return
        else:
            rospy.loginfo(result)
            rospy.logwarn('Failed to move to marker {}'.format(marker.id))


if __name__ == '__main__':
    main()