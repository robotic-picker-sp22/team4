#!/usr/bin/env python

import rospy
import control_msgs.msg
import trajectory_msgs.msg
import actionlib

ACTION_NAME = control_msgs.msg.FollowJointTrajectoryAction
JOINT_NAME = 'torso_lift_joint'

TIME_FROM_START = 5  # How many seconds it should take to set the torso height.


class Torso(object):
    """Torso controls the robot's torso height.
    """
    MIN_HEIGHT = 0.0
    MAX_HEIGHT = 0.4

    def __init__(self):
        self.client = actionlib.SimpleActionClient('torso_controller/follow_joint_trajectory', ACTION_NAME)
        self.client.wait_for_server()

    def set_height(self, height):
        """Sets the torso height.

        This will always take ~5 seconds to execute.

        Args:
            height: The height, in meters, to set the torso to. Values range
                from Torso.MIN_HEIGHT (0.0) to Torso.MAX_HEIGHT(0.4).
        """
        # TODO: Check that the height is between MIN_HEIGHT and MAX_HEIGHT.
        # TODO: Create a trajectory point
        # TODO: Set position of trajectory point
        # TODO: Set time of trajectory point
        if height < self.MIN_HEIGHT or height > self.MAX_HEIGHT:
            rospy.logerr('Invalid height inputted')
            return
        
        trajectory_pt = trajectory_msgs.msg.JointTrajectoryPoint()
        trajectory_pt.positions = [height]
        trajectory_pt.time_from_start = rospy.Duration(5)

        # TODO: Create goal
        # TODO: Add joint name to list
        # TODO: Add the trajectory point created above to trajectory
        goal = control_msgs.msg.FollowJointTrajectoryGoal()
        goal.trajectory = trajectory_msgs.msg.JointTrajectory()
        goal.trajectory.joint_names.append(JOINT_NAME)
        goal.trajectory.points.append(trajectory_pt)
        
        # TODO: Send goal
        # TODO: Wait for result
        self.client.send_goal(goal)
        self.client.wait_for_result()
        return self.client.get_result()
