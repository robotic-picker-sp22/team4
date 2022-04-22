#!/usr/bin/env python

from distutils.archive_util import make_zipfile
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, InteractiveMarkerFeedback
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PoseStamped
import rospy, copy
import robot_api
import math

GRIPPER_MESH = 'package://fetch_description/meshes/gripper_link.dae'
L_FINGER_MESH = 'package://fetch_description/meshes/l_gripper_finger_link.STL'
R_FINGER_MESH = 'package://fetch_description/meshes/r_gripper_finger_link.STL'

def make_gripper(pose_stamped : PoseStamped):
    FINGER_OFFSET = 0.0529
    int_marker = InteractiveMarker()
    int_marker.header = pose_stamped.header
    int_marker.pose = pose_stamped.pose

    gripper = Marker()
    gripper.type = Marker.MESH_RESOURCE
    gripper.mesh_resource = GRIPPER_MESH
    gripper.scale.x = 1
    gripper.scale.y = 1
    gripper.scale.z = 1
    gripper.color.a = 1
    gripper.color.g = 1
    gripper.pose.position.x = 0.166

    l_finger = Marker()
    l_finger.type = Marker.MESH_RESOURCE
    l_finger.mesh_resource = L_FINGER_MESH
    l_finger.pose.position.y = -FINGER_OFFSET
    l_finger.scale.x = 1
    l_finger.scale.y = 1
    l_finger.scale.z = 1
    l_finger.color.a = 1
    l_finger.color.g = 1
    l_finger.pose.position.x = 0.166


    r_finger = Marker()
    r_finger.type = Marker.MESH_RESOURCE
    r_finger.mesh_resource = R_FINGER_MESH
    r_finger.pose.position.y = FINGER_OFFSET
    r_finger.scale.x = 1
    r_finger.scale.y = 1
    r_finger.scale.z = 1
    r_finger.color.a = 1
    r_finger.color.g = 1
    r_finger.pose.position.x = 0.166


    ctl = InteractiveMarkerControl()
    ctl.always_visible = True
    ctl.markers.append(gripper)
    ctl.markers.append(l_finger)
    ctl.markers.append(r_finger)

    int_marker.controls.append(ctl)

    return int_marker

def make_6dof_controls():
    controls = []
    control = InteractiveMarkerControl()

    control.orientation_mode = InteractiveMarkerControl.FIXED
    control.orientation.w = 1;
    control.orientation.x = 1;
    control.orientation.y = 0;
    control.orientation.z = 0;
    control.name = "rotate_x";
    control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS;
    controls.append(control);

    control = copy.deepcopy(control)
    control.name = "move_x";
    control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS;
    controls.append(control);
    
    control = copy.deepcopy(control)
    control.orientation.w = 1;
    control.orientation.x = 0;
    control.orientation.y = 1;
    control.orientation.z = 0;
    control.name = "rotate_z";
    control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS;
    controls.append(control);

    control = copy.deepcopy(control)
    control.name = "move_z";
    control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS;
    controls.append(control);

    control = copy.deepcopy(control)
    control.orientation.w = 1;
    control.orientation.x = 0;
    control.orientation.y = 0;
    control.orientation.z = 1;
    control.name = "rotate_y";
    control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS;
    controls.append(control);

    control = copy.deepcopy(control)
    control.name = "move_y";
    control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS;
    controls.append(control);

    return controls

class GripperTeleop(object):
    def __init__(self, arm, gripper, im_server):
        self._arm = arm
        self._gripper = gripper
        self._im_server = im_server

    def start(self):
        init_pose = PoseStamped()
        init_pose.header.frame_id = 'wrist_roll_link'
        
        gripper_im = make_gripper(init_pose)
        gripper_im.name = 'gripper_teleop'
        gripper_im.controls.extend(make_6dof_controls())
        gripper_im.scale = 0.25
        self._im_server.insert(gripper_im, feedback_cb=self.handle_feedback)
        self._im_server.applyChanges()

    


    def handle_feedback(self, feedback):
        pass


class AutoPickTeleop(object):
    def __init__(self, arm, gripper, im_server):
        self._arm = arm
        self._gripper = gripper
        self._im_server = im_server

    def start(self):
        # obj_im = InteractiveMarker() ...
        self._im_server.insert(obj_im, feedback_cb=self.handle_feedback)

    def handle_feedback(self, feedback):
        pass


def main():
    rospy.init_node('gripper_im_server')
    arm = robot_api.Arm()
    gripper = robot_api.Gripper()
    im_server = InteractiveMarkerServer('gripper_im_server')
    #auto_pick_im_server = InteractiveMarkerServer('auto_pick_im_server')
    teleop = GripperTeleop(arm, gripper, im_server)
    #auto_pick = AutoPickTeleop(arm, gripper, auto_pick_im_server)
    teleop.start()
    #auto_pick.start()
    rospy.spin()

if __name__ == '__main__':
  main()