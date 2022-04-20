#!/usr/bin/env python

import rospy
import copy

from interactive_markers.interactive_marker_server import *
from interactive_markers.menu_handler import *
from visualization_msgs.msg import *
from geometry_msgs.msg import *
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Quaternion, Pose, Point, Vector3
from std_msgs.msg import Header, ColorRGBA

class MakePoseMarker:

    def __init__(self, name, pose=Pose(position=Point(0,0,0), orientation=Quaternion(0,0,0,1))):
        self.pose = pose
        self.name = name

    def makeName(self):
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = "map"
        int_marker.pose = copy.deepcopy(self.pose)
        int_marker.pose.position.z = 1.45
        int_marker.scale = 1

        int_marker.name = self.name + "_name"
        int_marker.description = ""

        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 1
        control.orientation.z = 0
        control.interaction_mode = InteractiveMarkerControl.NONE
        int_marker.controls.append(copy.deepcopy(control))

        marker = Marker(
                type=Marker.TEXT_VIEW_FACING,
                scale=Vector3(0.3, 0.3, 0.3),
                color=ColorRGBA(0.0, 1.0, 0.0, 0.8),
                text=self.name)

        # make a box which also moves in the plane
        control.markers.append( marker )
        control.always_visible = True
        int_marker.controls.append(control)
        return int_marker

    def makeArrow(self):
        box_marker = Marker()
        box_marker.type = Marker.ARROW
        box_marker.pose.orientation.w = 1
        box_marker.scale.x = 0.6
        box_marker.scale.y = 0.1
        box_marker.scale.z = 0.1
        box_marker.color.r = 0.0
        box_marker.color.g = 0.5
        box_marker.color.b = 0.5
        box_marker.color.a = 1.0

        return box_marker

    def makePoseMarker(self):
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = "map"
        int_marker.pose = self.pose
        int_marker.scale = 1

        int_marker.name = self.name
        int_marker.description = ""

        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 1
        control.orientation.z = 0
        control.interaction_mode = InteractiveMarkerControl.MOVE_ROTATE
        int_marker.controls.append(copy.deepcopy(control))

        # make a box which also moves in the plane
        control.markers.append( self.makeArrow() )
        control.always_visible = True
        int_marker.controls.append(control)

        # we want to use our special callback function
        return int_marker

        # set different callback for POSE_UPDATE feedback
        #server.setCallback(int_marker.name, alignMarker, InteractiveMarkerFeedback.POSE_UPDATE )
