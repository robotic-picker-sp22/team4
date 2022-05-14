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

# BTA and BTC to ATC
def to_offset_2(p1 : Pose, p2: Pose):
    position = p1.position
    orientation = p1.orientation
    bta = tft.quaternion_matrix([orientation.x, orientation.y, orientation.z, orientation.w])
    bta[0][3] = position.x
    bta[1][3] = position.y
    bta[2][3] = position.z
    atb = np.linalg.inv(bta)
    # bta -> atb

    position = p2.position
    orientation = p2.orientation
    btc = tft.quaternion_matrix([orientation.x, orientation.y, orientation.z, orientation.w])
    btc[0][3] = position.x
    btc[1][3] = position.y
    btc[2][3] = position.z
    # btc
    atc = atb @ btc
    atc_out = Pose()
    atc_out.position.x = atc[0][3]
    atc_out.position.y = atc[1][3]
    atc_out.position.z = atc[2][3]
    qt = tft.quaternion_from_matrix(atc)
    atc_out.orientation.x = qt[0]
    atc_out.orientation.y = qt[1]
    atc_out.orientation.z = qt[2]
    atc_out.orientation.w = qt[3]
    return atc_out

# ATB and BTC to ATC
def to_offset_3(p1 : Pose, p2: Pose):
    position = p1.position
    orientation = p1.orientation
    atb = tft.quaternion_matrix([orientation.x, orientation.y, orientation.z, orientation.w])
    atb[0][3] = position.x
    atb[1][3] = position.y
    atb[2][3] = position.z
    # atb

    position = p2.position
    orientation = p2.orientation
    btc = tft.quaternion_matrix([orientation.x, orientation.y, orientation.z, orientation.w])
    btc[0][3] = position.x
    btc[1][3] = position.y
    btc[2][3] = position.z
    # btc
    atc = atb @ btc
    atc_out = Pose()
    atc_out.position.x = atc[0][3]
    atc_out.position.y = atc[1][3]
    atc_out.position.z = atc[2][3]
    qt = tft.quaternion_from_matrix(atc)
    atc_out.orientation.x = qt[0]
    atc_out.orientation.y = qt[1]
    atc_out.orientation.z = qt[2]
    atc_out.orientation.w = qt[3]
    return atc_out