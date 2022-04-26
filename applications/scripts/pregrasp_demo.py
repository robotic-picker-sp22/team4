#! /usr/bin/env python
from geometry_msgs.msg import Pose, Point
import numpy as np
import tf.transformations as tft

def to_offset(pose : Pose, offset: Point):
    position = pose.position
    orientation = pose.orientation
    base_obj = tft.quaternion_matrix([orientation.x, orientation.y, orientation.z, orientation.w])
    base_obj[0][3] = position.x
    base_obj[1][3] = position.y
    base_obj[2][3] = position.z

    obj_grasp = tft.quaternion_matrix([0, 0, 0, 1])
    obj_grasp[0][3] += offset.x
    obj_grasp[1][3] += offset.y
    obj_grasp[2][3] += offset.z

    base_grasp = base_obj @ obj_grasp
    pregrasp = Pose()
    pregrasp.position.x = base_grasp[0][3]
    pregrasp.position.y = base_grasp[1][3]
    pregrasp.position.z = base_grasp[2][3]
    qt = tft.quaternion_from_matrix(base_grasp)
    pregrasp.orientation.x = qt[0]
    pregrasp.orientation.y = qt[1]
    pregrasp.orientation.z = qt[2]
    pregrasp.orientation.w = qt[3]
    return pregrasp
