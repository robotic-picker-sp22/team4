#!/usr/bin/env python

from distutils.archive_util import make_zipfile
from operator import is_
from tkinter import E
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, InteractiveMarkerFeedback
from interactive_markers.menu_handler import *
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PoseStamped, Point
from tf.listener import TransformListener
from moveit_msgs.msg import OrientationConstraint
from pregrasp_demo import to_offset
import rospy, copy
import robot_api
from joint_state_reader import JointStateReader
import math

GRIPPER_MESH = 'package://fetch_description/meshes/gripper_link.dae'
L_FINGER_MESH = 'package://fetch_description/meshes/l_gripper_finger_link.STL'
R_FINGER_MESH = 'package://fetch_description/meshes/r_gripper_finger_link.STL'

def make_gripper(offset : Point):
    FINGER_OFFSET = 0.0529
    

    gripper = Marker()
    gripper.type = Marker.MESH_RESOURCE
    gripper.mesh_resource = GRIPPER_MESH
    gripper.scale.x = 1
    gripper.scale.y = 1
    gripper.scale.z = 1
    gripper.color.a = 0.7
    gripper.color.g = 1
    gripper.pose.position.x = offset.x
    gripper.pose.position.y = offset.y
    gripper.pose.position.z = offset.z

    l_finger = Marker()
    l_finger.type = Marker.MESH_RESOURCE
    l_finger.mesh_resource = L_FINGER_MESH
    l_finger.scale.x = 1
    l_finger.scale.y = 1
    l_finger.scale.z = 1
    l_finger.color.a = 0.7
    l_finger.color.g = 1
    l_finger.pose.position.x = offset.x
    l_finger.pose.position.y = offset.y-FINGER_OFFSET
    l_finger.pose.position.z = offset.z

    r_finger = Marker()
    r_finger.type = Marker.MESH_RESOURCE
    r_finger.mesh_resource = R_FINGER_MESH
    r_finger.scale.x = 1
    r_finger.scale.y = 1
    r_finger.scale.z = 1
    r_finger.color.a = 0.7
    r_finger.color.g = 1
    r_finger.pose.position.x = offset.x
    r_finger.pose.position.y = offset.y + FINGER_OFFSET
    r_finger.pose.position.z = offset.z


    ctl = InteractiveMarkerControl()
    ctl.name = "gripper_mesh"
    ctl.always_visible = True
    ctl.markers.append(gripper)
    ctl.markers.append(l_finger)
    ctl.markers.append(r_finger)

    return ctl

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
        self._joint_listener = JointStateReader()
        rospy.sleep(0.1)

    def start(self):
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'wrist_roll_link'
        init_pose = PoseStamped()
        init_pose.header.frame_id = 'base_link'
        self._tf_listener.waitForTransform(goal_pose.header.frame_id, init_pose.header.frame_id, rospy.Time.now(),rospy.Duration(5.0))
        init_pose = self._tf_listener.transformPose(init_pose.header.frame_id, goal_pose)

        self.gripper_im = InteractiveMarker()
        self.gripper_im.header = init_pose.header
        self.gripper_im.pose = init_pose.pose
        self.gripper_im.controls.append(make_gripper(Point(x=0.166, y=0, z=0)))
        self.gripper_im.controls.extend(make_6dof_controls())
        self.gripper_im.name = 'gripper_teleop'
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
            names = robot_api.ArmJoints.names()
            joints = self._joint_listener.get_joints(names)
            ps = PoseStamped(pose=copy.deepcopy(feedback.pose))
            ps.header.frame_id = feedback.header.frame_id
            is_valid = self._arm.compute_ik(ps, joint_name=names, joint_pos=joints)
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

                names = robot_api.ArmJoints.names()
                joints = self._joint_listener.get_joints(names)
                ps = PoseStamped(pose=copy.deepcopy(feedback.pose))
                ps.header.frame_id = feedback.header.frame_id
                joints = self._arm.compute_ik(ps, joint_name=names, joint_pos=joints, nums=True)
                rospy.loginfo(self._arm.move_to_joint(names, joints, replan=True))

            elif feedback.menu_entry_id == 2:
                self._gripper.open()
            elif feedback.menu_entry_id == 3:
                self._gripper.close()


class AutoPickTeleop(object):
    def __init__(self, arm, gripper, im_server):
        self._arm = arm
        self._gripper = gripper
        self._im_server = im_server
        self._menu_handler = MenuHandler()
        self._tf_listener = TransformListener()
        self._joint_listener = JointStateReader()
        rospy.sleep(0.1)
        

    def start(self):
        self.make_im()
        self.valid = True
        self.make_menu()
        self._im_server.insert(self.obj_im, feedback_cb=self.handle_feedback)
        self._im_server.applyChanges()

    def make_im(self):
        # Make Initial Marker Pose
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'wrist_roll_link'
        init_pose = PoseStamped()
        init_pose.header.frame_id = 'base_link'
        self._tf_listener.waitForTransform(goal_pose.header.frame_id, init_pose.header.frame_id, rospy.Time.now(),rospy.Duration(5.0))
        init_pose = self._tf_listener.transformPose(init_pose.header.frame_id, goal_pose)
        init_pose.pose.position.x += 0.25
        
        self.obj_im = InteractiveMarker()
        self.obj_im.header = init_pose.header
        self.obj_im.pose = init_pose.pose

        self.offsets = []
        self.offsets.append(Point(-0.1, 0, 0)) # Pregrasp
        self.offsets.append(Point(0, 0, 0))    # Grasp
        self.offsets.append(None)              # Grip
        self.offsets.append(Point(-0.2, 0, 0)) # Lift

        for offset in self.offsets:
            if offset is not None:
                self.obj_im.controls.append(make_gripper(offset))
        self.obj_im.controls.extend(make_6dof_controls())
        
        box_marker = Marker()
        box_marker.type = Marker.CUBE
        box_marker.scale.x = 0.065
        box_marker.scale.y = 0.065
        box_marker.scale.z = 0.065
        box_marker.color.r = 1.0
        box_marker.color.g = 1.0
        box_marker.color.b = 0.0
        box_marker.color.a = 0.7

        # create a non-interactive control which contains the box
        box_control = InteractiveMarkerControl()
        box_control.always_visible = True
        box_control.markers.append( box_marker )
        self.obj_im.controls.append(box_control)


        self.obj_im.name = 'auto_pick_teleop'
        self.obj_im.scale = 0.25
    
    def make_menu(self):
        ctl = InteractiveMarkerControl()
        ctl.interaction_mode = InteractiveMarkerControl.MENU
        ctl.name = "gripper_menu"
        ctl.always_visible = True
        self.obj_im.controls.append(ctl)

        self.obj_im.menu_entries.append(MenuEntry(id=1, title="Pick from front"))
        self.obj_im.menu_entries.append(MenuEntry(id=2, title="Open gripper"))


    def handle_feedback(self, feedback):
        if feedback.event_type == InteractiveMarkerFeedback.POSE_UPDATE:
            change = False

            i = 0
            names = robot_api.ArmJoints.names()
            joints = self._joint_listener.get_joints(names)
            for ctl in self.obj_im.controls:
                if ctl.name == 'gripper_mesh':
                    if self.offsets[i] is None:
                        i+= 1
                    offset = self.offsets[i]
                    ps = PoseStamped(pose=to_offset(feedback.pose, Point(x=offset.x-0.166, y=offset.y, z=offset.z)))
                    ps.header.frame_id = feedback.header.frame_id
                    is_valid = self._arm.compute_ik(ps, joint_name=names, joint_pos=joints)
                    if is_valid and ctl.markers[0].color.g == 0:
                        change = True
                        for marker in ctl.markers:
                            marker.color.g = 1
                            marker.color.r = 0
                    elif not is_valid and ctl.markers[0].color.g == 1:
                        change = True
                        for marker in ctl.markers:
                            marker.color.g = 0
                            marker.color.r = 1
                    i += 1
                
            if change:
                self.obj_im.pose = feedback.pose
                self._im_server.insert(self.obj_im, feedback_cb=self.handle_feedback)
                self._im_server.applyChanges()
            
        elif feedback.event_type == InteractiveMarkerFeedback.MENU_SELECT:
            if feedback.menu_entry_id == 1:
                for ctl in self.obj_im.controls:
                    if ctl.name == 'gripper_mesh' and ctl.markers[0].color.g == 0:
                        return
                
                names = robot_api.ArmJoints.names()
                joints = self._joint_listener.get_joints(names)
                for offset in self.offsets:
                    if offset is None:
                        self._gripper.close()
                    else:
                        ps = PoseStamped(pose=to_offset(feedback.pose, Point(x=offset.x-0.166, y=offset.y, z=offset.z)))
                        ps.header.frame_id = feedback.header.frame_id
                        joints = self._arm.compute_ik(ps, joint_name=names, joint_pos=joints, nums=True)
                        rospy.loginfo(self._arm.move_to_joint(names, joints, replan=True))
                
                # oc = None
                # for offset in self.offsets:
                #     if offset is None:
                #         self._gripper.close()
                #     else:
                #         ps = PoseStamped(pose=to_offset(feedback.pose, Point(x=offset.x-0.166, y=offset.y, z=offset.z)))
                #         ps.header.frame_id = feedback.header.frame_id
                #         if self._arm.check_pose(ps, orientation_constraint=oc) == 'SUCCESS':
                #             rospy.loginfo(oc)
                #             rospy.loginfo(self._arm.move_to_pose(ps, orientation_constraint=oc, replan=True))
                #         else:
                #             rospy.loginfo("Failed")
                #         if oc is None:
                #             oc = OrientationConstraint()
                #             oc.header.frame_id = 'base_link'
                #             oc.link_name = 'wrist_roll_link'
                #             oc.orientation.w = 1
                #             oc.absolute_x_axis_tolerance = 0.4
                #             oc.absolute_y_axis_tolerance = 0.4
                #             oc.absolute_z_axis_tolerance = 3.14
                #             oc.weight = 1.0

            elif feedback.menu_entry_id == 2:
                self._gripper.open()


def main():
    rospy.init_node('gripper_im_server')
    arm = robot_api.Arm()
    gripper = robot_api.Gripper()
    im_server = InteractiveMarkerServer('gripper_im_server', q_size=2)
    teleop = GripperTeleop(arm, gripper, im_server)
    teleop.start()

    # rospy.init_node('auto_pick_im_server')
    # arm = robot_api.Arm()
    # gripper = robot_api.Gripper()
    auto_pick_im_server = InteractiveMarkerServer('auto_pick_im_server', q_size=2)
    auto_pick = AutoPickTeleop(arm, gripper, auto_pick_im_server)
    auto_pick.start()

    rospy.spin()

if __name__ == '__main__':
  main()