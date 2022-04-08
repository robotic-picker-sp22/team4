#!/usr/bin/env python

import math
import rospy
import control_msgs.msg
import trajectory_msgs.msg
import actionlib

LOOK_AT_ACTION_NAME = control_msgs.msg.PointHeadAction
PAN_TILT_ACTION_NAME = control_msgs.msg.FollowJointTrajectoryAction
PAN_JOINT = 'head_pan_joint'
TILT_JOINT = 'head_tilt_joint'
PAN_TILT_TIME = 2.5  # How many seconds it should take to move the head.


class Head(object):
    """Head controls the Fetch's head.

    It provides two interfaces:
        head.look_at(frame_id, x, y, z)
        head.pan_tilt(pan, tilt) # In radians

    For example:
        head = robot_api.Head()
        head.look_at('base_link', 1, 0, 0.3)
        head.pan_tilt(0, math.pi/4)
    """
    MIN_PAN = -math.pi/2  # TODO: Minimum pan angle, in radians.
    MAX_PAN = math.pi/2  # TODO: Maximum pan angle, in radians.
    MIN_TILT = -math.pi/4  # TODO: Minimum tilt angle, in radians.
    MAX_TILT = math.pi/2  # TODO: Maximum tilt angle, in radians.

    def __init__(self):
        self.look_client = actionlib.SimpleActionClient('head_controller/point_head', LOOK_AT_ACTION_NAME)
        self.look_client.wait_for_server()
        self.pantilt_client = actionlib.SimpleActionClient('head_controller/follow_joint_trajectory', PAN_TILT_ACTION_NAME)
        self.pantilt_client.wait_for_server()

    def look_at(self, frame_id, x, y, z):
        """Moves the head to look at a point in space.

        Args:
            frame_id: The name of the frame in which x, y, and z are specified.
            x: The x value of the point to look at.
            y: The y value of the point to look at.
            z: The z value of the point to look at.
        """
        goal = control_msgs.msg.PointHeadGoal()
        goal.target.header.frame_id = frame_id
        goal.target.point.x = x
        goal.target.point.y = y
        goal.target.point.z = z
        goal.min_duration = rospy.Duration(1)
        
        self.look_client.send_goal(goal)
        self.look_client.wait_for_result()
        return self.look_client.get_result()

    def pan_tilt(self, pan, tilt):
        """Moves the head by setting pan/tilt angles.

              Args:
            pan: The pan angle, in radians. A positive value is clockwise.
            tilt: The tilt angle, in radians. A positive value is downwards.
        """
        if pan < self.MIN_PAN or tilt < self.MIN_TILT or pan > self.MAX_PAN or tilt > self.MAX_TILT:
            rospy.logerr('Invalid Pan/Tilt Inputted')
            return
        
        trajectory_pt = trajectory_msgs.msg.JointTrajectoryPoint()
        trajectory_pt.positions = [pan, tilt]
        trajectory_pt.time_from_start = rospy.Duration(2.5)

        goal = control_msgs.msg.FollowJointTrajectoryGoal()
        goal.trajectory.joint_names.append(PAN_JOINT)
        goal.trajectory.joint_names.append(TILT_JOINT)
        goal.trajectory.points.append(trajectory_pt)

        self.pantilt_client.send_goal(goal)
        self.pantilt_client.wait_for_result()
        return self.pantilt_client.get_result()