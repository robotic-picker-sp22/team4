#! /usr/bin/env python

from pregrasp_demo import to_offset, to_offset_2, to_offset_3
import rospy
from robot_controllers_msgs.msg import QueryControllerStatesGoal, ControllerState
from robot_controllers_msgs.msg import QueryControllerStatesAction
import actionlib
from moveit_msgs.msg import MoveItErrorCodes, MoveGroupAction
import os
import json
import robot_api
from joint_state_reader import JointStateReader
from ar_track_alvar_msgs.msg import AlvarMarkers
from tf.listener import TransformListener
from geometry_msgs.msg import Pose, PoseStamped


in_robot = os.getenv("ROBOT") == None

class ArTagReader(object):
    def __init__(self):
        self.markers = []
        rospy.Subscriber('/ar_pose_marker', AlvarMarkers, self.callback)
        while len(self.markers) == 0:
            rospy.sleep(0.1)

    def callback(self, msg):
        self.markers = msg.markers

class Action:
    def __init__(self, input_dict):
        if 'gripper_open' in input_dict:
            self.type = 'gripper'
            self.gripper_open = input_dict['gripper_open']
        elif input_dict['frame'] == -1:
            self.type = 'base'
            self.joints = input_dict['joints']
        else:
            self.type = input_dict['frame']
            self.pose = Pose()
            self.pose.position.x = input_dict['pose']['pos_x']
            self.pose.position.y = input_dict['pose']['pos_y']
            self.pose.position.z = input_dict['pose']['pos_z']
            self.pose.orientation.x = input_dict['pose']['ori_x']
            self.pose.orientation.y = input_dict['pose']['ori_y']
            self.pose.orientation.z = input_dict['pose']['ori_z']
            self.pose.orientation.w = input_dict['pose']['ori_w']
        self.in_dict = input_dict

    def to_dict(self):
        return self.in_dict
        # LOL SO BAD
        

class ProgByDemon:
    def __init__(self) -> None:
        self._controller_client = actionlib.SimpleActionClient('query_controller_states', QueryControllerStatesAction)
        self.actions = []
        self.gripper = robot_api.Gripper()
        self.arm = robot_api.Arm()
        self.reader = JointStateReader()
        self.tag_reader = ArTagReader()
        self.joint_names = robot_api.ArmJoints.names()
        self.tf_listener = TransformListener()
        rospy.sleep(0.5)

    def relax(self, isRelax):
        print(f"Changing robot relaxed state to {isRelax}")
        goal = QueryControllerStatesGoal()
        state = ControllerState()
        state.name = 'arm_controller/follow_joint_trajectory'

        if isRelax:
            state.state = ControllerState.STOPPED
        else:
            state.state = ControllerState.RUNNING
        
        goal.updates.append(state)
        self._controller_client.send_goal(goal)
        self._controller_client.wait_for_result(rospy.Duration(10))

    def save(self, path):
        with open(path, 'w') as f:
            json.dump([a.to_dict() for a in self.actions], f, indent=4)
        self.actions = []
    
    def execute(self, path):
        with open(path, 'r') as f:
            self.actions = [Action(a) for a in json.load(f)]
        i = 1

        for action in self.actions:
            if action.type == 'gripper':
                if action.gripper_open:
                    self.gripper.open()
                    ac = "Gripper Opened"
                else:
                    self.gripper.close()
                    ac = "Gripper Closed"
            elif action.type == 'base':
                if self.arm.move_to_joint(self.joint_names, action.joints) != 'SUCCESS':
                    print(f'Action {i} Failed: Move relative to base!')
                    break
                ac = "Moved to pose relative to base"
            else:
                tag_pose = None
                for marker in self.tag_reader.markers:
                    if marker.id == action.type:
                        tag_pose = marker.pose.pose
                        break
                if tag_pose == None:
                    print(f"Action {i} Failed: Tag {action.type} not found!")
                    break
                wrist_goal = PoseStamped(pose=to_offset_3(tag_pose, action.pose))
                wrist_goal.header.frame_id = 'base_link'
                # print(action.pose, '\n', tag_pose, '\n', wrist_goal.pose)
                joints = self.reader.get_joints(self.joint_names)
                joints = self.arm.compute_ik(wrist_goal, joint_name=self.joint_names, joint_pos=joints, nums=True)
                if joints == False:
                    print(f"Action {i} Failed: Compute IK failed!")
                    break
                
                if self.arm.move_to_joint(self.joint_names, joints) != 'SUCCESS':
                    print(f'Action {i} Failed: Move to Joint failed!')
                    break

                ac = f"Move to pose relative to tag {action.type}"
            print(f"Action {i} Completed: {ac}")
            i += 1

            # Stored: Tag1,t1->Wrist,t1
            # Tag1,t1->Wrist,t1 == Tag1,t2->Wrist,t2
            # B->Tag1,t2 * Tag1,t2->Wrist,t2
            # = B->Wrist,t2

    def open(self):
        self.gripper.open()
        self.actions.append(Action({'gripper_open' : True}))

    def close(self):
        self.gripper.close(self.gripper.MAX_EFFORT)
        self.actions.append(Action({'gripper_open' : False}))

    def add_action(self, tag_id):
        if tag_id == -1:
            self.actions.append(Action({'frame' : -1, 'joints' : self.reader.get_joints(self.joint_names)}))
        else:
            tag_pose = None
            for marker in self.tag_reader.markers:
                if marker.id == tag_id:
                    tag_pose = marker.pose.pose
                    break
            if tag_pose == None:
                print(f"Tag {tag_id} not found! No pose saved.")
                return
            self.tf_listener.waitForTransform('wrist_roll_link', 'base_link',  rospy.Time.now(),rospy.Duration(5.0))
            wrist_pose = PoseStamped()
            wrist_pose.header.frame_id = 'wrist_roll_link'
            wrist_pose = self.tf_listener.transformPose('base_link', wrist_pose).pose
            final = to_offset_2(tag_pose, wrist_pose)
            # print(wrist_pose, '\n', tag_pose, '\n', final)
            # Have Base->Wrist and Base->Tag. Need Tag->Wrist to store.
            self.actions.append(Action({
                'frame' : tag_id,
                'pose' : {'pos_x': final.position.x, 'pos_y': final.position.y, 'pos_z':final.position.z,
                           'ori_x': final.orientation.x,
                           'ori_y': final.orientation.y,
                           'ori_z': final.orientation.z,
                           'ori_w': final.orientation.w}
            }))
    
    def list_tag_ids(self):
        return [marker.id for marker in self.tag_reader.markers]

def wait_for_time():
    """Wait for simulated time to begin.
    """
    while rospy.Time().now().to_sec() == 0:
        pass

def help(a = None):
    print("     create_prog: create a program.\n" +
          "     save_prog: save a program.\n" + 
          "     execute_prog: execute a program.\n" +
          "     save_pose: Save the robots current pose. Overwrites if <name> already exists.\n" + 
          "     open_grip: Open the gripper.\n" +
          "     close_grip: Close the gripper.\n" +
          "     help: Show this list of commands\n" +
          "     quit: Quit"
          )

def save_pose(server):
    print("Relative to base frame (b) or tag (t)?")
    
    print(">", end=" ")
    inp = input()
    while inp != 'b' and inp != 't':
        print("Please enter b (base) or t (tag):")
        print(">", end=" ")
        inp = input()
    
    if inp == 'b':
        server.add_action(-1)
    else:
        tags = server.list_tag_ids()
        print("Here are your tag options: ")
        print(tags)
        print("Choose a tag from the above")
        print(">", end=" ")
        tag = None
        while tag is None:
            try:
                tag = int(input())
            except:
                print("Choose an integer tag from the following:")
                print(tags)
                print(">", end=" ")
        server.add_action(tag)

def open_serv(server):
    server.open()

def close_serv(server):
    server.close()

def create_prog(server):
    print("Creating program")
    if in_robot:
        server.relax(True)
    print("Created!")

def save_prog(server):
    print("Save program to which file?")
    print(">", end=" ")
    path = input()
    server.save(path)
    if in_robot:
        server.relax(False)
    print("Saved!")

def execute_prog(server):
    print("Execute which program?")
    print(">", end=" ")
    path = input()
    server.execute(path)
    print("Executed!")


def main():
    rospy.init_node('prog_by_demon')
    wait_for_time()

    help()

    server = ProgByDemon()

    cmd2action = {
        "create_prog": create_prog,
        "save_prog": save_prog,
        "save_pose": save_pose,
        "open_grip": open_serv,
        "close_grip": close_serv,
        "help": help,
        "execute_prog": execute_prog
    }
    print(">", end=" ")
    while True:
        command = input()

        if command == "quit":
            break
        elif command not in cmd2action:
            print("Bad command!!! Please use help to see the commands")
            print(">", end=" ")
        else:
            print("-----------------------------------")
            cmd2action[command](server)
            print("-----------------------------------")
            print(">", end=" ")



if __name__ == "__main__":
    main()

