#!/usr/bin/env python
import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Quaternion, Pose, Point, Vector3
from std_msgs.msg import Header, ColorRGBA
from nav_msgs.msg import Odometry


class NavPath(object):
    def __init__(self):
        self._path = []
        self._publisher = rospy.Publisher('visualization_marker', Marker, queue_size=5)
    
    def callback(self, msg):
        rospy.loginfo(msg)
        if len(self._path) == 0 or self.dist(self._path[-1], msg.pose.pose.position):
            self._path.append(msg.pose.pose.position)
        if len(self._path) > 1:
            self.publish()

    def dist(self, P1, P2):
        d = (P1.x - P2.x) ** 2 + (P1.y - P2.y) ** 2 + (P1.z - P2.z) ** 2
        return d > 0.3

    def publish(self):
        marker = Marker(
                    type=Marker.LINE_STRIP,
                    id=27,
                    pose=Pose(Point(0, 0, 0), Quaternion(0, 0, 0, 0)),
                    points=self._path,
                    scale=Vector3(0.1, 0, 0),
                    header=Header(frame_id='odom'),
                    color=ColorRGBA(0.0, 1.0, 0.0, 0.8),
                    text="")
        self._publisher.publish(marker)


def main():
    rospy.init_node('my_node')
    nav_path = NavPath()
    rospy.sleep(0.5)
    rospy.Subscriber('odom', Odometry, nav_path.callback)
    rospy.spin()

if __name__ == '__main__':
  main()