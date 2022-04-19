#!/usr/bin/env python

import rospy
import copy

from interactive_markers.interactive_marker_server import *
from interactive_markers.menu_handler import *
from visualization_msgs.msg import *
from geometry_msgs.msg import *

class MakePoseMarker:
    global pose, frame_id

    def __init__(self, name, pose=Pose(position=Point(0,0,0), orientation=Quaternion(0,0,0,1))):
        self.pose = pose
        self.name = name
        
    def processFeedback(self, feedback ):
        s = "Feedback from marker '" + feedback.marker_name
        s += "' / control '" + feedback.control_name + "'"

        mp = ""
        if feedback.mouse_point_valid:
            mp = " at " + str(feedback.mouse_point.x)
            mp += ", " + str(feedback.mouse_point.y)
            mp += ", " + str(feedback.mouse_point.z)
            mp += " in frame " + feedback.header.frame_id
        pose = feedback.pose
        frame_id = feedback.header.frame_id


        if feedback.event_type == InteractiveMarkerFeedback.BUTTON_CLICK:
            rospy.loginfo( s + ": button click" + mp + "." )
        elif feedback.event_type == InteractiveMarkerFeedback.MENU_SELECT:
            rospy.loginfo( s + ": menu item " + str(feedback.menu_entry_id) + " clicked" + mp + "." )
        elif feedback.event_type == InteractiveMarkerFeedback.POSE_UPDATE:
            rospy.loginfo( s + ": pose changed")
        elif feedback.event_type == InteractiveMarkerFeedback.MOUSE_DOWN:
            rospy.loginfo( s + ": mouse down" + mp + "." )
        elif feedback.event_type == InteractiveMarkerFeedback.MOUSE_UP:
            rospy.loginfo( s + ": mouse up" + mp + "." )
        ########
        rospy.loginfo("xxxx ")
        rospy.loginfo(feedback.pose)
        rospy.loginfo(feedback.header.frame_id)
        #server.applyChanges()

    # def alignMarker( feedback ):
    #     pose = feedback.pose

    #     pose.position.x = round(pose.position.x-0.5)+0.5
    #     pose.position.y = round(pose.position.y-0.5)+0.5

    #     rospy.loginfo( feedback.marker_name + ": aligning position = " + str(feedback.pose.position.x) + "," + str(feedback.pose.position.y) + "," + str(feedback.pose.position.z) + " to " +
    #                                                                      str(pose.position.x) + "," + str(pose.position.y) + "," + str(pose.position.z) )

    #     server.setPose( feedback.marker_name, pose )
    #     server.applyChanges()

    def makeArrow(self, msg ):
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
        int_marker.description = self.name

        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 1
        control.orientation.z = 0
        control.interaction_mode = InteractiveMarkerControl.MOVE_ROTATE
        int_marker.controls.append(copy.deepcopy(control))

        # make a box which also moves in the plane
        control.markers.append( self.makeArrow(int_marker) )
        control.always_visible = True
        int_marker.controls.append(control)

        # we want to use our special callback function
        return int_marker

        # set different callback for POSE_UPDATE feedback
        #server.setCallback(int_marker.name, alignMarker, InteractiveMarkerFeedback.POSE_UPDATE )
