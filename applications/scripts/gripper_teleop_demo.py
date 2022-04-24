#!/usr/bin/env python

from distutils.archive_util import make_zipfile
from operator import is_
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, InteractiveMarkerFeedback
from interactive_markers.menu_handler import *
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PoseStamped
from tf.listener import TransformListener
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
    ctl.name = "gripper_mesh"
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
    control.orientation.w = 1
    control.orientation.x = 1
    control.orientation.y = 0
    control.orientation.z = 0
    control.name = "rotate_x"
    control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
    controls.append(control)

    control = copy.deepcopy(control)
    control.name = "move_x"
    control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
    controls.append(control)
    
    control = copy.deepcopy(control)
    control.orientation.w = 1
    control.orientation.x = 0
    control.orientation.y = 1
    control.orientation.z = 0
    control.name = "rotate_z"
    control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
    controls.append(control)

    control = copy.deepcopy(control)
    control.name = "move_z"
    control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
    controls.append(control)

    control = copy.deepcopy(control)
    control.orientation.w = 1
    control.orientation.x = 0
    control.orientation.y = 0
    control.orientation.z = 1
    control.name = "rotate_y"
    control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
    controls.append(control)

    control = copy.deepcopy(control)
    control.name = "move_y"
    control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
    controls.append(control)

    return controls

class GripperTeleop(object):
    def __init__(self, arm, gripper, im_server):
        self._arm = arm
        self._gripper = gripper
        self._im_server = im_server
        self._menu_handler = MenuHandler()
        self._tf_listener = TransformListener()
        rospy.sleep(0.1)

    def start(self):
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'wrist_roll_link'
        init_pose = PoseStamped()
        init_pose.header.frame_id = 'base_link'
        self._tf_listener.waitForTransform(goal_pose.header.frame_id, init_pose.header.frame_id, rospy.Time.now(),rospy.Duration(5.0))
        init_pose = self._tf_listener.transformPose(init_pose.header.frame_id, goal_pose)
        self.gripper_im : InteractiveMarker = make_gripper(init_pose)
        self.gripper_im.name = 'gripper_teleop'
        self.gripper_im.controls.extend(make_6dof_controls())
        self.gripper_im.scale = 0.25
        self.valid = True
        self.make_menu()
        self._im_server.insert(self.gripper_im, feedback_cb=self.handle_feedback)
        self._im_server.applyChanges()

    def make_menu(self):
        ctl = InteractiveMarkerControl()
        ctl.interaction_mode = InteractiveMarkerControl.MENU
        ctl.name = "gripper_menu"
        ctl.always_visible = True
        self.gripper_im.controls.append(ctl)

        self.gripper_im.menu_entries.append(MenuEntry(id=1, title="Move gripper here"))
        self.gripper_im.menu_entries.append(MenuEntry(id=2, title="Open gripper"))
        self.gripper_im.menu_entries.append(MenuEntry(id=3, title="Close gripper"))


    def handle_feedback(self, feedback:InteractiveMarkerFeedback):
        if feedback.event_type == InteractiveMarkerFeedback.POSE_UPDATE:
            ps = PoseStamped(pose=copy.deepcopy(feedback.pose))
            ps.header.frame_id = feedback.header.frame_id
            is_valid = self._arm.compute_ik(ps)
            if is_valid and not self.valid:
                for ctl in self.gripper_im.controls:
                    if ctl.name == 'gripper_mesh':
                        for marker in ctl.markers:
                            marker.color.g = 1
                            marker.color.r = 0
                self.gripper_im.pose = feedback.pose
                self._im_server.insert(self.gripper_im, feedback_cb=self.handle_feedback)
                self._im_server.applyChanges()
            elif not is_valid and self.valid:
                for ctl in self.gripper_im.controls:
                    if ctl.name == 'gripper_mesh':
                        for marker in ctl.markers:
                            marker.color.g = 0
                            marker.color.r = 1
                self.gripper_im.pose = feedback.pose
                self._im_server.insert(self.gripper_im, feedback_cb=self.handle_feedback)
                self._im_server.applyChanges()
            self.valid = is_valid
            
        elif feedback.event_type == InteractiveMarkerFeedback.MENU_SELECT:
            if feedback.menu_entry_id == 1:
                if not self.valid:
                    return
                ps = PoseStamped(pose=copy.deepcopy(feedback.pose))
                ps.header.frame_id = feedback.header.frame_id
                rospy.loginfo(self._arm.move_to_pose(ps))

            elif feedback.menu_entry_id == 2:
                self._gripper.open()
            elif feedback.menu_entry_id == 3:
                self._gripper.close()


class AutoPickTeleop(object):
    def __init__(self, arm, gripper, im_server):
        self._arm = arm
        self._gripper = gripper
        self._im_server = im_server

    def start(self):
        obj_im = InteractiveMarker()
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