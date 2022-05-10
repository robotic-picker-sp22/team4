#! /usr/bin/env python

import copy
from geometry_msgs.msg import PoseStamped, Quaternion
from ar_track_alvar_msgs.msg import AlvarMarker, AlvarMarkers
import robot_api
import rospy
import tf.transformations as tft
import math

def wait_for_time():
    """Wait for simulated time to begin.
    """
    while rospy.Time().now().to_sec() == 0:
        pass


class SmartCropper(object):
    def __init__(self, bin=False):
        self.markers = {}
        marker = AlvarMarker()
        self.markers[15] = marker
        marker.pose.pose.position.x = 0.9242293709590111
        marker.pose.pose.position.y = 0.37773913844132134
        marker.pose.pose.position.z = 1.3045296669006348

        marker.pose.pose.orientation.x = 0.02906246775568839
        marker.pose.pose.orientation.y = -0.6873829019389252
        marker.pose.pose.orientation.z = -0.04345797533317163
        marker.pose.pose.orientation.w = 0.724411156367648
        
        mat = tft.quaternion_matrix([marker.pose.pose.orientation.x,
                                        marker.pose.pose.orientation.y,
                                        marker.pose.pose.orientation.z,
                                        marker.pose.pose.orientation.w])
        self.init_rot = math.atan2(mat[1, 0], mat[2, 0]) * 180 / math.pi

        if bin == False:
            rospy.set_param("crop_min_x", 0.9)
            rospy.set_param("crop_max_x", 1.4)
            rospy.set_param("crop_min_y", -0.47)
            rospy.set_param("crop_max_y", 0.5)
            rospy.set_param("crop_min_z", 0.77)
            rospy.set_param("crop_max_z", 1.6)
        else:
            rospy.set_param("crop_min_x", 0.9)
            rospy.set_param("crop_max_x", 1.4)
            rospy.set_param("crop_min_y", 0.1)
            rospy.set_param("crop_max_y", 0.5)
            rospy.set_param("crop_min_z", 1.16)
            rospy.set_param("crop_max_z", 1.34)
        rospy.set_param("crop_rot_x", 0)
        rospy.set_param("crop_rot_y", 0)
        rospy.set_param("crop_rot_z", 0)

    def callback(self, msg):
        # TODO: refer each AR tag by id and average the change across all AR tags.
        need_transform = True
        for marker in msg.markers:
            if marker.id == 15:
                need_transform = False
                diff_x = (marker.pose.pose.position.x - self.markers[marker.id].pose.pose.position.x)
                diff_y = (marker.pose.pose.position.y - self.markers[marker.id].pose.pose.position.y)
                diff_z = (marker.pose.pose.position.z - self.markers[marker.id].pose.pose.position.z)


                diff_rx = (marker.pose.pose.orientation.x - self.markers[marker.id].pose.pose.orientation.x)
                diff_ry = (marker.pose.pose.orientation.y - self.markers[marker.id].pose.pose.orientation.y)
                diff_rz = (marker.pose.pose.orientation.z - self.markers[marker.id].pose.pose.orientation.z)
                diff_rw = (marker.pose.pose.orientation.w - self.markers[marker.id].pose.pose.orientation.w)
                
                mat = tft.quaternion_matrix([marker.pose.pose.orientation.x,
                                marker.pose.pose.orientation.y,
                                marker.pose.pose.orientation.z,
                                marker.pose.pose.orientation.w])
                new_rot = math.atan2(mat[1, 0], mat[2, 0]) * 180 / math.pi

                rospy.set_param("crop_min_x", rospy.get_param("crop_min_x") + diff_x)
                rospy.set_param("crop_max_x", rospy.get_param("crop_max_x") + diff_x)
                rospy.set_param("crop_min_y", rospy.get_param("crop_min_y") + diff_y)
                rospy.set_param("crop_max_y", rospy.get_param("crop_max_y") + diff_y)
                rospy.set_param("crop_min_z", rospy.get_param("crop_min_z") + diff_z)
                rospy.set_param("crop_max_z", rospy.get_param("crop_max_z") + diff_z)
                
                #rospy.set_param("crop_rot_x", rospy.get_param("crop_rot_x") + diff_rx)
                #rospy.set_param("crop_rot_y", rospy.get_param("crop_rot_y") + diff_ry)
                #print(new_rot, self.init_rot, marker.id)
                #rospy.set_param("crop_rot_x", 1.5)

                rospy.sleep(0.1)
            self.markers[marker.id] = marker


def main():
    rospy.init_node('smart_cropper')
    wait_for_time()
                                                                               
    reader = SmartCropper(bin=True)
    sub = rospy.Subscriber('/ar_pose_marker', AlvarMarkers, reader.callback) # Subscribe to AR tag poses, use reader.callback
    
    while len(reader.markers) == 0:
        rospy.sleep(0.1)

    rate = rospy.Rate(2)
    while not rospy.is_shutdown():
        rate.sleep()

if __name__ == '__main__':
    main()