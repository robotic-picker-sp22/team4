#!/usr/bin/env python

import pickle
from warnings import catch_warnings
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, Pose
from std_msgs.msg import Header
from map_annotator.msg import PoseNames
from map_annotator.msg import UserAction
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, InteractiveMarkerFeedback
from visualization_msgs.msg import Marker

from geometry_msgs.msg import Point
from marker import MakePoseMarker


FILE_NAME = "/home/capstone/catkin_ws/src/cse481sp22/map_annotator/frontend/pose_data.pkl"

def wait_for_time():
    """Wait for simulated time to begin.
    """
    while rospy.Time().now().to_sec() == 0:
        pass

class MapAnnotatorServer(object):
    def __init__(self):
        # rospy.init_node('backend', anonymous=True)
        # wait_for_time()

        self._amcl_sub = rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, callback=self.handleMessage)
        self._goal_pub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=10)
        self._pose_names_pub = rospy.Publisher("/map_annotator/pose_names", PoseNames, latch=True, queue_size=10)
        self._interactive_server = InteractiveMarkerServer("map_annotator/map_poses")
        
        try:
            self.load(FILE_NAME)
            self._pose_names_pub.publish(PoseNames(pose_names=list(self._record.keys())))
        except:
            self._record = {}
            self.dump(FILE_NAME)
            self._pose_names_pub.publish(PoseNames(pose_names=list(self._record.keys())))

        self.current_pose = Pose()
        self.current_frame = ""
        for pose_name in self._record:
            new_marker = MakePoseMarker(name=pose_name, pose=self._record[pose_name].pose)
            self._interactive_server.insert(new_marker.makePoseMarker(), self.save)
            self._interactive_server.applyChanges()
            pass
    
    def handleMessage(self, msg):
        self.current_pose = msg.pose.pose
        self.current_frame = msg.header.frame_id

    def create(self, request):
        new_marker = MakePoseMarker(name=request)
        self._interactive_server.insert(new_marker.makePoseMarker(), self.save)
        self._interactive_server.applyChanges()

    def save(self, request):
        name = request.marker_name
        self.load(FILE_NAME)
        self._record[name] = PoseStamped(header=Header(frame_id=request.header.frame_id), pose=request.pose)
        self.dump(FILE_NAME)
        self._pose_names_pub.publish(PoseNames(pose_names=list(self._record.keys())))
        #self.createInteractiveMarker(request)
        return 200

    def delete(self, request):
        try:
            self.load(FILE_NAME)
            self._record.pop(request)
            self.dump(FILE_NAME)
            self._pose_names_pub.publish(PoseNames(pose_names=list(self._record.keys())))
            self._interactive_server.erase(request)
            self._interactive_server.applyChanges()
            return 200
        except:
            return 400

    def goto(self, request):
        #move_base_simple/goal
        try:
            self.load(FILE_NAME)
            self._goal_pub.publish(self._record[request])
            return 200
        except KeyError:
            return 401

    def list(self, request):
        self.load(FILE_NAME)
        print("\n".join(self._record.keys()))
        return 200

    def createInteractiveMarker(self, name):
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = self.current_frame
        int_marker.name = name
        int_marker.description = name
        
        int_marker.pose = self.current_pose
        
        int_marker.pose.orientation.x = 0
        int_marker.pose.orientation.y = 1
        int_marker.pose.orientation.z = 0
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

        control = InteractiveMarkerControl()
        control.interaction_mode = InteractiveMarkerControl.MOVE_PLANE
        control.always_visible = True
        control.markers.append(box_marker)
        int_marker.controls.append(control)

        #self._interactive_server.insert(int_marker, self.handle_viz_pose_input)

    def handle_viz_pose_input(self):
        pass

    def load(self, filename):
        with open(filename, 'rb') as file:
            self._record = pickle.load(file)

    def dump(self, filename):
        with open(filename, 'wb') as file:
            pickle.dump(self._record, file, protocol=pickle.HIGHEST_PROTOCOL)

    def handleIncomingAction(self, req):
        cmd = req.command

        if cmd == req.DELETE:
            self.delete(req.name)
        elif cmd == req.CREATE:
            self.create(req.name)
        elif cmd == req.GOTO:
            self.goto(req.name)
    
def main():
    rospy.init_node('map_annotator_server')
    wait_for_time()

    server = MapAnnotatorServer()

    rospy.Subscriber("/map_annotator/user_actions", UserAction, callback=server.handleIncomingAction)
    
    rospy.spin()


if __name__ == "__main__":
    main()
