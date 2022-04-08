import rospy

from .arm_joints import ArmJoints
import control_msgs.msg
import trajectory_msgs.msg
import actionlib


class Arm(object):
    """Arm controls the robot's arm.

    Joint space control:
        joints = ArmJoints()
        # Fill out joint states
        arm = robot_api.Arm()
        arm.move_to_joints(joints)
    """

    def __init__(self):
        self.client = actionlib.SimpleActionClient('arm_controller/follow_joint_trajectory', control_msgs.msg.FollowJointTrajectoryAction)
        self.client.wait_for_server()

    def move_to_joints(self, arm_joints):
        """Moves the robot's arm to the given joints.

        Args:
            arm_joints: An ArmJoints object that specifies the joint values for
                the arm.
        """
        trajectory_pt = trajectory_msgs.msg.JointTrajectoryPoint()
        trajectory_pt.positions = arm_joints.values()
        trajectory_pt.time_from_start = rospy.Duration(5)

        # TODO: Create goal
        # TODO: Add joint name to list
        # TODO: Add the trajectory point created above to trajectory
        goal = control_msgs.msg.FollowJointTrajectoryGoal()
        goal.trajectory = trajectory_msgs.msg.JointTrajectory()
        goal.trajectory.joint_names = arm_joints.names()
        goal.trajectory.points.append(trajectory_pt)

        self.client.send_goal(goal)
        self.client.wait_for_result()
        return self.client.get_result()

