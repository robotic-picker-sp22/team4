#!/usr/bin/env python

import robot_api
import rospy
from web_teleop.srv import SetTorso, SetTorsoResponse
from web_teleop.srv import SetHead, SetHeadResponse
from web_teleop.srv import Gripper, GripperResponse
from web_teleop.srv import Arm, ArmResponse


def wait_for_time():
    """Wait for simulated time to begin.
    """
    while rospy.Time().now().to_sec() == 0:
        pass


class ActuatorServer(object):
    def __init__(self):
        self._torso = robot_api.Torso()
        self._head = robot_api.Head()
        self._gripper = robot_api.Gripper()
        self._arm = robot_api.Arm()

    def handle_set_torso(self, request):
        self._torso.set_height(request.height)
        return SetTorsoResponse()

    def handle_move_head(self, request):
        self._head.pan_tilt(request.pan, request.tilt)
        return SetHeadResponse()

    def handle_gripper(self, request):
        if request.grab == float(1):
            self._gripper.close()
        else:
            self._gripper.open()

        return GripperResponse()

    def handle_arm_move(self, request):
        arm = robot_api.ArmJoints()
        arm.set_shoulder_pan(request.shoulder_pan)
        arm.set_shoulder_lift(request.shoulder_lift)
        arm.set_upperarm_roll(request.upperarm_roll)
        arm.set_elbow_flex(request.elbow_flex)
        arm.set_forearm_roll(request.forearm_roll)
        arm.set_wrist_flex(request.wrist_flex)
        arm.set_wrist_roll(request.wrist_roll)
        
        self._arm.move_to_joints(arm)
        return ArmResponse()

def main():
    rospy.init_node('web_teleop_actuators')
    wait_for_time()
    server = ActuatorServer()
    torso_service = rospy.Service('web_teleop/set_torso', SetTorso,
                                  server.handle_set_torso)
    head_service = rospy.Service('web_teleop/set_head', SetHead, 
                                 server.handle_move_head)
    gripper_service = rospy.Service('web_teleop/gripper', Gripper,
                                    server.handle_gripper)
    arm_service = rospy.Service('web_teleop/arm', Arm,
                                server.handle_arm_move)

    rospy.spin()


if __name__ == '__main__':
    main()