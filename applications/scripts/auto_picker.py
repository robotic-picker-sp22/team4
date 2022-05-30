#! /usr/bin/env python

import sys
import time
import robot_api
import joint_state_reader
import copy
import rospy
from bin_marker_cropper import SmartCropper
from geometry_msgs.msg import PoseStamped, Point, Quaternion

HORIZONTAL_GRIPPER = Quaternion(x=0, y=0, z=0, w=1)
VERTICAL_GRIPPER = Quaternion(x=0, y=0, z=0, w=1)

GRIPPER_SIZE=0.166

# TODO: Fill these in
BASE_JOINTS = [
            0.8955447257751464,
            0.8731829323425293,
            0.12912157249668121,
            -2.2634818531762697,
            -0.7929600692779541,
            1.3032064565795898,
            -3.076633191946411
        ]
ABOVE_BIN_JOINTS = [
            0.16613693845214844,
            1.571527639831543,
            0.12873807621219635,
            -1.4266952254067993,
            -0.23535784254379272,
            0.8913326391357422,
            -3.076633191946411
        ]

PREGRASP_OFFSET = Point(x=-GRIPPER_SIZE-0.30,y=0,z=0)
GRASP_OFFSET = Point(x=-GRIPPER_SIZE,y=0,z=0)


def help():
    print("Needs a file for the shelf configuration")

def adjust_pose(pose, point, orientation=None):
    new_pose = copy.deepcopy(pose)
    new_pose.position.x += point.x
    new_pose.position.y += point.y
    new_pose.position.z += point.z
    if orientation is not None:
        new_pose.orientation = orientation
    new_pose = PoseStamped(pose=new_pose)
    new_pose.header.frame_id = 'base_link'
    return new_pose

class AutoPicker():
    def __init__(self, objects, debug=False):
        self.cropper = SmartCropper(debug)
        self.arm = robot_api.Arm()
        self.gripper = robot_api.Gripper()
        self.joint_names = robot_api.ArmJoints.names()
        self.joint_listener = joint_state_reader.JointStateReader()
        self.objects = objects
        self.detected_objects = {}
        self.detected_objects_conf = {}
        self.object_sub = rospy.Subscriber('/segment_cloud', PoseStamped, self.process_data)
    
    def process_data(self, data):
        if (data.pose.position.x >= rospy.get_param("crop_min_x") and data.pose.position.x <= rospy.get_param("crop_max_x")
    	     and data.pose.position.y >= rospy.get_param("crop_min_y") and data.pose.position.y <= rospy.get_param("crop_max_y")
    	     and data.pose.position.z >= rospy.get_param("crop_min_z") and data.pose.position.z <= rospy.get_param("crop_max_z")):
            if (data.header.frame_id not in self.detected_objects_conf
                or data.header.stamp.secs >= self.detected_objects_conf[data.header.frame_id]):
                    self.detected_objects[data.header.frame_id] = data.pose
                    self.detected_objects_conf[data.header.frame_id] = data.header.stamp.secs
    
    def move_to_new_pose(self, new_pose, curr_joints=None):
        if curr_joints is None:
            curr_joints = self.joint_listener.get_joints(self.joint_names)
        new_joints = self.arm.compute_ik(new_pose, joint_name=self.joint_names, joint_pos=curr_joints, nums=True)
        # print(new_joints)
        if new_joints is None:
            return None
        return self.move_to_new_joints(new_joints, curr_joints)

    def move_to_new_joints(self, new_joints, curr_joints=None):
        if curr_joints is None:
            curr_joints = self.joint_listener.get_joints(self.joint_names)
        result = self.arm.move_to_joint(robot_api.ArmJoints.from_list(new_joints))
        if result != 'SUCCESS':
            print(result)
            return None  
        return new_joints

    def pick_up(self, pose, orientation, gripper_effort=100):
        # BASE
        joints = self.move_to_new_joints(BASE_JOINTS)
        if joints is None:
            print("Basic Action Failed!")
            return False

        # OPEN GRIPPER
        self.gripper.open()

        # MOVE TO PREGRASP
        new_pose = adjust_pose(pose, PREGRASP_OFFSET)
        joints = self.move_to_new_pose(new_pose, joints)
        if joints is None:
            print("Pregrasp Action Failed!")
            return False

        # MOVE TO GRASP
        new_pose = adjust_pose(pose, GRASP_OFFSET)
        joints = self.move_to_new_pose(new_pose, joints)
        if joints is None:
            print("Grasp Action Failed!")
            return False

        # CLOSE GRIPPER
        self.gripper.close(gripper_effort)

        # TODO: MOVE TO POSTGRASP
        new_pose = adjust_pose(pose, PREGRASP_OFFSET)
        joints = self.move_to_new_pose(new_pose, joints)
        if joints is None:
            print("Postgrasp Action Failed!")
            return False

        # MOVE TO ABOVE BIN
        joints = self.move_to_new_joints(ABOVE_BIN_JOINTS, joints)

        # OPEN GRIPPER
        self.gripper.open()
        return True
    
    def find_object(self, obj):
        self.detected_objects = {}
        self.detected_objects_conf = {}
        rospy.sleep(3)
        return (self.detected_objects.get(obj), HORIZONTAL_GRIPPER, 100)

    def pick_object(self, obj, bin_col, bin_row):
        # Crop to bin_row, bin_col
        print(f"Cropping to Bin ({bin_col}, {bin_row})")
        self.cropper.crop_to_bin(bin_col, bin_row)
        print(f"Finding Object {obj} in bin")
        pose, orientation, effort = self.find_object(obj)
        if pose is None:
            print(f"Object {obj} not found!")
            self.cropper.crop_to_shelf() 
            return
        # Pick up object
        print(f"Picking up Object")
        success = self.pick_up(pose, orientation, effort)
        # TODO: Save gripper effort/direction for each object
        # Change the objects dict
        if success:
            del self.objects[obj]
        # Crop back once successful
        self.cropper.crop_to_shelf() 

def wait_for_time():
    """Wait for simulated time to begin.
    """
    while rospy.Time().now().to_sec() == 0:
        pass

def main():
    if len(sys.argv) < 2:
        help()
        return
    with open(sys.argv[1], 'r') as config:
        objects = {}
        for bin in config.readlines():
            bin_info = bin.split(' ')
            bin_col = int(bin_info[0])
            bin_row = int(bin_info[1])
            for obj in bin_info[2:]:
                objects[obj.strip()] = (bin_col, bin_row)
    rospy.init_node('smart_cropper')
    wait_for_time()
    ap = AutoPicker(objects, debug=True)
    print("Beginning program...\nType quit to quit\n")
    while True:
        print("Select which object to pick:", list(objects.keys()))
        print(">", end=" ")
        obj_name = input()

        if obj_name == "quit":
            break
        elif obj_name not in ap.objects:
            print("Object " + obj_name + " does not exist")
        else:
            print("-----------------------------------")
            ap.pick_object(obj_name, *objects[obj_name])
            print("-----------------------------------")

if __name__ == "__main__":
    main()