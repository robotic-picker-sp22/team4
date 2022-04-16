#!/usr/bin/env python

from xmlrpc.client import Marshaller

from yaml import Mark
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, InteractiveMarkerFeedback
from visualization_msgs.msg import Marker
import rospy
import robot_api
import math

def handle_viz_input_forward(input):
    if (input.event_type == InteractiveMarkerFeedback.BUTTON_CLICK):
        rospy.loginfo(input.marker_name + ' was clicked.')
        base = robot_api.Base()
        base.go_forward(0.5)
    else:
        pass
        # rospy.loginfo('Cannot handle this InteractiveMarker event')

def handle_viz_input_back(input):
    if (input.event_type == InteractiveMarkerFeedback.BUTTON_CLICK):
        rospy.loginfo(input.marker_name + ' was clicked.')
        base = robot_api.Base()
        base.go_forward(-0.5)
    else:
        pass
        # rospy.loginfo('Cannot handle this InteractiveMarker event')

def handle_viz_input_clockwise(input):
    if (input.event_type == InteractiveMarkerFeedback.BUTTON_CLICK):
        rospy.loginfo(input.marker_name + ' was clicked.')
        base = robot_api.Base()
        base.turn(30 * math.pi / 180)
    else:
        pass
        # rospy.loginfo('Cannot handle this InteractiveMarker event')

def handle_viz_input_counter_clockwise(input):
    if (input.event_type == InteractiveMarkerFeedback.BUTTON_CLICK):
        rospy.loginfo(input.marker_name + ' was clicked.')
        base = robot_api.Base()
        base.turn((30 * math.pi / 180) * -1)
    else:
        pass
        # rospy.loginfo('Cannot handle this InteractiveMarker event')        

def main():
    rospy.init_node('base_marker')

    server = InteractiveMarkerServer("base_marker")

    # Forward Arrow
    int_marker = InteractiveMarker()
    int_marker.header.frame_id = "base_link"
    int_marker.name = "forward_marker"
    int_marker.description = "Go Straight"
    int_marker.pose.position.x = 0.5
    int_marker.pose.position.z = 0.2
    int_marker.pose.orientation.w = 1

    box_marker = Marker()
    box_marker.type = Marker.ARROW
    box_marker.pose.orientation.w = 1
    box_marker.scale.x = 0.4
    box_marker.scale.y = 0.1
    box_marker.scale.z = 0.1
    box_marker.color.r = 0.0
    box_marker.color.g = 0.5
    box_marker.color.b = 0.5
    box_marker.color.a = 1.0

    button_control = InteractiveMarkerControl()
    button_control.interaction_mode = InteractiveMarkerControl.BUTTON
    button_control.always_visible = True
    button_control.markers.append(box_marker)
    int_marker.controls.append(button_control)

    server.insert(int_marker, handle_viz_input_forward)
    
    # Counter-Clockwise Arrow
    int_marker = InteractiveMarker()
    int_marker.header.frame_id = "base_link"
    int_marker.name = "counter_clockwise_marker"
    int_marker.description = "Turn right"
    int_marker.pose.position.y = -0.5
    int_marker.pose.position.z = 0.2
    int_marker.pose.orientation.w = 1
    int_marker.pose.orientation.z = -1

    box_marker = Marker()
    box_marker.type = Marker.ARROW
    box_marker.pose.orientation.w = 1
    box_marker.scale.x = 0.4
    box_marker.scale.y = 0.1
    box_marker.scale.z = 0.1
    box_marker.color.r = 0.0
    box_marker.color.g = 0.5
    box_marker.color.b = 0.5
    box_marker.color.a = 1.0

    button_control = InteractiveMarkerControl()
    button_control.interaction_mode = InteractiveMarkerControl.BUTTON
    button_control.always_visible = True
    button_control.markers.append(box_marker)
    int_marker.controls.append(button_control)

    server.insert(int_marker, handle_viz_input_counter_clockwise)

    # Clockwise Arrow
    int_marker = InteractiveMarker()
    int_marker.header.frame_id = "base_link"
    int_marker.name = "clockwise_marker"
    int_marker.description = "Turn left"
    int_marker.pose.position.y = 0.5
    int_marker.pose.position.z = 0.2
    int_marker.pose.orientation.w = 1
    int_marker.pose.orientation.z = 1

    box_marker = Marker()
    box_marker.type = Marker.ARROW
    box_marker.pose.orientation.w = 1
    box_marker.scale.x = 0.4
    box_marker.scale.y = 0.1
    box_marker.scale.z = 0.1
    box_marker.color.r = 0.0
    box_marker.color.g = 0.5
    box_marker.color.b = 0.5
    box_marker.color.a = 1.0

    button_control = InteractiveMarkerControl()
    button_control.interaction_mode = InteractiveMarkerControl.BUTTON
    button_control.always_visible = True
    button_control.markers.append(box_marker)
    int_marker.controls.append(button_control)

    server.insert(int_marker, handle_viz_input_clockwise)

    # Backward arrow
    int_marker = InteractiveMarker()
    int_marker.header.frame_id = "base_link"
    int_marker.name = "backward_marker"
    int_marker.description = "Back up"
    int_marker.pose.position.x = -0.5
    int_marker.pose.position.z = 0.2
    int_marker.pose.orientation.w = 0
    int_marker.pose.orientation.y = 1

    box_marker = Marker()
    box_marker.type = Marker.ARROW
    box_marker.pose.orientation.w = 1
    box_marker.scale.x = 0.4
    box_marker.scale.y = 0.1
    box_marker.scale.z = 0.1
    box_marker.color.r = 0.0
    box_marker.color.g = 0.5
    box_marker.color.b = 0.5
    box_marker.color.a = 1.0

    button_control = InteractiveMarkerControl()
    button_control.interaction_mode = InteractiveMarkerControl.BUTTON
    button_control.always_visible = True
    button_control.markers.append(box_marker)
    int_marker.controls.append(button_control)

    server.insert(int_marker, handle_viz_input_back)

    server.applyChanges()

    rospy.spin()


if __name__ == '__main__':
  main()