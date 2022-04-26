#! /usr/bin/env python

from moveit_python import PlanningSceneInterface
import robot_api
import rospy


def wait_for_time():
    """Wait for simulated time to begin.
    """
    while rospy.Time().now().to_sec() == 0:
        pass


def print_usage():
    print('Usage: rosrun applications lab26_obstacles.py')
    print('Drive the robot until the PlanningScene lines up with the point cloud.')


def main():
    rospy.init_node('lab26_obstacles')
    wait_for_time()

    planning_scene = PlanningSceneInterface('base_link')
    planning_scene.clear()
    planning_scene.removeCollisionObject('table')
    planning_scene.removeCollisionObject('floor')
    planning_scene.removeCollisionObject('cube')
    planning_scene.addBox('floor', 2, 2, 0.01, 0, 0, 0.01/2)
    planning_scene.addBox('table', 1.1, 1.5, 0.83, 1.1, 0, 0.72/2)
    planning_scene.setColor('table', 1, 0, 0)
    planning_scene.addBox('cube', 0.07, 0.07, 0.07, 0.785, 0.245, 0.8)
    planning_scene.sendColors()


    rospy.sleep(2)


if __name__ == '__main__':
    main()