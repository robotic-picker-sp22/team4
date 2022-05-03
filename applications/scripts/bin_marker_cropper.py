#! /usr/bin/env python

import copy
from geometry_msgs.msg import PoseStamped
from ar_track_alvar_msgs.msg import AlvarMarkers
import robot_api
import rospy


def wait_for_time():
    """Wait for simulated time to begin.
    """
    while rospy.Time().now().to_sec() == 0:
        pass


class SmartCropper(object):
    def __init__(self):
        self.markers = []

    def callback(self, msg):
        # TODO: refer each AR tag by id and average the change across all AR tags.
        if len(self.markers) == 0:
            diff_x = (msg.markers[0].pose.pose.position.x - self.markers[0].pose.pose.position.x)
            diff_y = (msg.markers[0].pose.pose.position.y - self.markers[0].pose.pose.position.y)
            diff_z = (msg.markers[0].pose.pose.position.z - self.markers[0].pose.pose.position.z)
            rospy.set_param("crop_min_x", rospy.get_param("crop_min_x") + diff_x)
            rospy.set_param("crop_max_x", rospy.get_param("crop_max_x") + diff_x)
            rospy.set_param("crop_min_y", rospy.get_param("crop_min_y") + diff_y)
            rospy.set_param("crop_max_y", rospy.get_param("crop_max_y") + diff_y)
            rospy.set_param("crop_min_z", rospy.get_param("crop_min_z") + diff_z)
            rospy.set_param("crop_max_z", rospy.get_param("crop_max_z") + diff_z)
        self.markers = msg.markers


def main():
    rospy.init_node('smart_cropper')
    wait_for_time()
                                                                               
    reader = SmartCropper()
    sub = rospy.Subscriber('/ar_pose_marker', AlvarMarkers, reader.callback) # Subscribe to AR tag poses, use reader.callback
    
    while len(reader.markers) == 0:
        rospy.sleep(0.1)

    rate = rospy.Rate(2)
    while not rospy.is_shutdown():
        rate.sleep()

if __name__ == '__main__':
    main()