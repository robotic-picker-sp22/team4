#! /usr/bin/env python

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


in_robot = os.getenv("ROBOT") is not None

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

    def execute(self):
        pass

    def to_dict(self):
        pass

class ProgByDemon:
    def __init__(self) -> None:
        self._controller_client = actionlib.SimpleActionClient('query_controller_states', QueryControllerStatesAction)
        self.actions = []
        self.gripper = robot_api.Gripper()
        self.reader = JointStateReader()
        self.tag_reader = ArTagReader()
        self.joint_names = robot_api.ArmJoints.names()
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
            json.dump([a.to_dict() for a in self.actions], path)
        self.actions = []
    
    def execute(self, path):
        with open(path, 'r') as f:
            self.actions = [Action(a) for a in json.load(f)]
        for action in self.actions:
            action.execute() #TODO: Add arguments
            # Stored: Tag1,t1->Wrist,t1
            # Tag1,t1->Wrist,t1 == Tag1,t2->Wrist,t2
            # B->Tag1,t2 * Tag1,t2->Wrist,t2
            # = B->Wrist,t2

    def open(self):
        self.gripper.open()
        self.actions.append[Action({'gripper_open' : True})]

    def close(self):
        self.gripper.close(self.gripper.MAX_EFFORT)
        self.actions.append[Action({'gripper_open' : False})]

    def add_action(self, frame):
        if frame == -1:
            self.actions.append(Action({'frame' : -1, 'joints' : self.reader.get_joints(self.joint_names)}))
        else:
            frame_marker = None
            for marker in self.tag_reader.markers:
                if marker.id == frame:
                    frame_marker = marker
                    break

            self.actions.append(Action({
                'frame' : frame,

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

def open(server):
    server.open()

def close(server):
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
        "open_grip": open,
        "close_grip": close,
        "help": help,
        "execute_prog": execute_prog
    }

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

