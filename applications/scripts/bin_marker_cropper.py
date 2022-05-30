#! /usr/bin/env python

import copy

from matplotlib.ft2font import HORIZONTAL
from geometry_msgs.msg import Point, PoseStamped, Quaternion
from ar_track_alvar_msgs.msg import AlvarMarker, AlvarMarkers
import robot_api
import rospy
import tf.transformations as tft
import math
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Quaternion, Pose, Point, Vector3
from std_msgs.msg import Header, ColorRGBA

def wait_for_time():
    """Wait for simulated time to begin.
    """
    while rospy.Time().now().to_sec() == 0:
        pass

bins = {
    # TODO: Fill this out 
#     0 : [("crop_min_x", 0.9),   ("crop_max_x", 1.17),
#          ("crop_min_y", 0.15), ("crop_max_y", 0.45),
#          ("crop_min_z", 1.34),  ("crop_max_z", 1.515)],
#     1 : [("crop_min_x", 0.9),   ("crop_max_x", 1.17),
#          ("crop_min_y", 0.015), ("crop_max_y", 0.145),
#          ("crop_min_z", 1.38),  ("crop_max_z", 1.515)]
# }

    0 : [("crop_min_x", 0.8),   ("crop_max_x", 1.17),
         ("crop_min_y", 0.12), ("crop_max_y", 0.48),
         ("crop_min_z", 1.32),  ("crop_max_z", 1.545)],
    1 : [("crop_min_x", 0.8),   ("crop_max_x", 1.17),
         ("crop_min_y", -0.015), ("crop_max_y", 0.175),
         ("crop_min_z", 1.35),  ("crop_max_z", 1.545)]
}

VERTICAL_OFFSET_LARGE = -0.186 # 1.34 - 1.154 = 0.186
VERTICAL_OFFSET_SMALL = -0.117  # 1.38 - 1.262 = 0.118
HORIZONTAL_OFFSET_SMALL = -0.145 # 0.015 - (-0.13)

shelf = [("crop_min_x", 0.9),   ("crop_max_x", 1.17),
         ("crop_min_y", -0.405), ("crop_max_y", 0.45),
         ("crop_min_z", 0.80),  ("crop_max_z", 1.515)] # TODO: Fill this out

tag_positions = {
    5 : Point(x=0.9559044882103249,y=-0.019223203422662653,z=1.474956973099414),
    15 : Point(x=0.9274771007490746,y=0.400968141025967,z=1.485398021745093),
    4 : Point(x=0.9666865453308011,y=-0.3658890558613671,z=1.51348822057983)
}

tag_orientations = {    0 : [("crop_min_x", 0.9),   ("crop_max_x", 1.17),
         ("crop_min_y", 0.15), ("crop_max_y", 0.45),
         ("crop_min_z", 1.34),  ("crop_max_z", 1.515)],
    1 : [("crop_min_x", 0.9),   ("crop_max_x", 1.17),
         ("crop_min_y", 0.015), ("crop_max_y", 0.145),
         ("crop_min_z", 1.38),  ("crop_max_z", 1.515)]
}

VERTICAL_OFFSET_LARGE = -0.186 # 1.34 - 1.154 = 0.186
VERTICAL_OFFSET_SMALL = -0.117  # 1.38 - 1.262 = 0.118
HORIZONTAL_OFFSET_SMALL = -0.145 # 0.015 - (-0.13)

shelf = [("crop_min_x", 0.9),   ("crop_max_x", 1.17),
         ("crop_min_y", -0.405), ("crop_max_y", 0.45),
         ("crop_min_z", 0.80),  ("crop_max_z", 1.515)] # TODO: Fill this out

#     5 : Quaternion(x=0.7598432407518454,y=-0.028215889277710477,z=-0.6491402477358514,w=0.021425495220381455),
#     15 : Quaternion(x=0.02216194372208134,y=-0.7094829045379218,z=-0.010448754710440687,w=0.7042965852138531),
#     4 : Quaternion(x=-0.4644964420162109,y=-0.7094829045379218,z=-0.010448754710440687,w=0.7042965852138531)
# }

SHELF_OFFSET = {"crop_min_x": 0,   
                "crop_max_x": -0.02,
                "crop_min_y": 0.005,
                "crop_max_y": -0.005,
                "crop_min_z": 0.001,
                "crop_max_z": -0.001}

class SmartCropper(object):
    def __init__(self, debug=False):
        self.debug = debug
        self.pub_marker = rospy.Publisher('visualization_marker', Marker, queue_size=5)
        rospy.sleep(1)
        self.crop_to_shelf()

    def draw_box(self):
        if self.debug:
            min_x = rospy.get_param("crop_min_x")
            min_y = rospy.get_param("crop_min_y")
            min_z = rospy.get_param("crop_min_z")
            max_x = rospy.get_param("crop_max_x")
            max_y = rospy.get_param("crop_max_y")
            max_z = rospy.get_param("crop_max_z")
            marker = Marker(
                            type=Marker.CUBE,
                            id=1000,
                            pose=Pose(Point((max_x + min_x)/2, (max_y + min_y)/2, (max_z + min_z)/2), Quaternion(0, 0, 0, 1)),
                            scale=Vector3(max_x - min_x, max_y - min_y, max_z - min_z),
                            header=Header(frame_id='base_link'),
                            color=ColorRGBA(0.0, 0, 1, 0.5))

            self.pub_marker.publish(marker)

    
    def reset_tag(self):
        self.markers = {}
        marker = AlvarMarker()
        for marker in tag_positions.keys():
            mk = AlvarMarker()
            self.markers[marker] = mk
            mk.pose.pose.position = tag_positions[marker]
            mk.pose.pose.orientation = tag_orientations[marker]
        
        # mat = tft.quaternion_matrix([marker.pose.pose.orientation.x,
        #                                 marker.pose.pose.orientation.y,
        #                                 marker.pose.pose.orientation.z,
        #                                 marker.pose.pose.orientation.w])
        # self.init_rot = math.atan2(mat[1, 0], mat[2, 0]) * 180 / math.pi

    def crop_to_bin(self, bin_col, bin_row):
        self.reset_tag()
        for param, val in bins[0 if bin_col == 0 else 1]:
            value = val + SHELF_OFFSET[param]

            if bin_col == 0:
                rospy.set_param(param, value + (VERTICAL_OFFSET_LARGE * bin_row if param[-1] == 'z' else 0))
            else:
                rospy.set_param(param, value + (VERTICAL_OFFSET_SMALL * bin_row if param[-1] == 'z' 
                else (HORIZONTAL_OFFSET_SMALL * (bin_col - 1) if param[-1] == 'y' else 0)))
        self.draw_box()
            
    
    def crop_to_shelf(self):
        self.reset_tag()
        for param, val in shelf:
            rospy.set_param(param, val) 
        self.draw_box()

    def callback(self, msg):
        # TODO: refer each AR tag by id and average the change across all AR tags.
        need_transform = True
        new_markers = {}
        for marker in msg.markers:
            if need_transform and marker.id in self.markers:            
                need_transform = False
                diff_x = (marker.pose.pose.position.x - self.markers[marker.id].pose.pose.position.x)
                diff_y = (marker.pose.pose.position.y - self.markers[marker.id].pose.pose.position.y)
                diff_z = (marker.pose.pose.position.z - self.markers[marker.id].pose.pose.position.z)
                
                diff_rx = (marker.pose.pose.orientation.x - self.markers[marker.id].pose.pose.orientation.x)
                diff_ry = (marker.pose.pose.orientation.y - self.markers[marker.id].pose.pose.orientation.y)
                diff_rz = (marker.pose.pose.orientation.z - self.markers[marker.id].pose.pose.orientation.z)
                diff_rw = (marker.pose.pose.orientation.w - self.markers[marker.id].pose.pose.orientation.w)
                
                rospy.set_param("crop_min_x", rospy.get_param("crop_min_x") + diff_x)
                rospy.set_param("crop_max_x", rospy.get_param("crop_max_x") + diff_x)
                rospy.set_param("crop_min_y", rospy.get_param("crop_min_y") + diff_y)
                rospy.set_param("crop_max_y", rospy.get_param("crop_max_y") + diff_y)
                rospy.set_param("crop_min_z", rospy.get_param("crop_min_z") + diff_z)
                rospy.set_param("crop_max_z", rospy.get_param("crop_max_z") + diff_z)
                self.draw_box()
            # mat = tft.quaternion_matrix([marker.pose.pose.orientation.x,
            #     marker.pose.pose.orientation.y,
            #     marker.pose.pose.orientation.z,
            #     marker.pose.pose.orientation.w])
            # new_rot = math.atan2(mat[1, 0], mat[2, 0]) * 180 / math.pi
            #rospy.set_param("crop_rot_x", rospy.get_param("crop_rot_x") + diff_rx)
            #rospy.set_param("crop_rot_y", rospy.get_param("crop_rot_y") + diff_ry)
            #print(new_rot, self.init_rot, marker.id)
            #rospy.set_param("crop_rot_x", 1.5)
            rospy.sleep(0.1)
            new_markers[marker.id] = marker
        if not need_transform:
            self.markers = new_markers


def main():
    rospy.init_node('smart_cropper')
    wait_for_time()
                                                                               
    reader = SmartCropper()
    sub = rospy.Subscriber('/ar_pose_marker', AlvarMarkers, reader.callback) # Subscribe to AR tag poses, use reader.callback

    print("hello 1")
    
    while len(reader.markers) == 0:
        rospy.sleep(0.1)

    rate = rospy.Rate(2)
    while not rospy.is_shutdown():
        rate.sleep()

if __name__ == '__main__':
    main()
