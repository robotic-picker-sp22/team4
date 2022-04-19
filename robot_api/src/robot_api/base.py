#! /usr/bin/env python

# TODO: import ????????_msgs.msg
from distutils.command.config import config
from matplotlib.pyplot import spring
import rospy
from geometry_msgs.msg import Twist, Pose2D, Quaternion
from nav_msgs.msg import Odometry
import tf.transformations as tft
import numpy as np
import math
import copy

TO_DEGREE = 180 / math.pi

class Base(object):
    """Base controls the mobile base portion of the Fetch robot.

    Sample usage:
        base = robot_api.Base()
        while CONDITION:
            base.move(0.2, 0)
        base.stop()
    """

    def __init__(self):
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self._odom_sub = rospy.Subscriber('odom', Odometry, callback=self._odom_callback)

    def _odom_callback(self, msg):
        self.current_pose = Pose2D()

        self.current_pose.x = msg.pose.pose.position.x
        self.current_pose.y = msg.pose.pose.position.y
        
        self.quaternion_vals = Quaternion()
        self.quaternion_vals.x = msg.pose.pose.orientation.x
        self.quaternion_vals.y = msg.pose.pose.orientation.y
        self.quaternion_vals.z = msg.pose.pose.orientation.z
        self.quaternion_vals.w = msg.pose.pose.orientation.w

        self.current_pose.theta = self.quaternion_to_yaw(self.quaternion_vals)

    def quaternion_to_yaw(self, q):
        m = tft.quaternion_matrix([q.x, q.y, q.z, q.w])

        x = m[0, 0]
        y = m[1, 0]

        theta_rads = math.atan2(y , x) % (2 * math.pi)

        roll, pitch, yaw = tft.euler_from_quaternion([q.x, q.y, q.z, q.w])

        return theta_rads


    def go_forward(self, distance, speed=0.1):
        """Moves the robot a certain distance.

        It's recommended that the robot move slowly. If the robot moves too
        quickly, it may overshoot the target. Note also that this method does
        not know if the robot's path is perturbed (e.g., by teleop). It stops
        once the distance traveled is equal to the given distance or more.

        Args:
            distance: The distance, in meters, to move. A positive value
                means forward, negative means backward.
            speed: The speed to travel, in meters/second.
        """
        # TODO: rospy.sleep until the base has received at least one message on /odom
        # TODO: record start position, use Python's copy.deepcopy
        rospy.sleep(1)
        start = copy.deepcopy(self.current_pose)
        rate = rospy.Rate(10)
        # TODO: CONDITION should check if the robot has traveled the desired distance
        # TODO: Be sure to handle the case where the distance is negative!
        curr_trav_dist = 0
        while abs(curr_trav_dist) < abs(distance):
            # TODO: you will probably need to do some math in this loop to check the CONDITION
            direction = -1 if distance < 0 else 1
            step = direction * speed
            self.move(step, 0)
            curr_trav_dist = ((self.current_pose.x - start.x) ** 2 + (self.current_pose.y - start.y) ** 2) ** 0.5
            rate.sleep()

    def turn(self, angular_distance, speed=0.5):
        """Rotates the robot a certain angle.

        Args:
            angular_distance: The angle, in radians, to rotate. A positive
                value rotates counter-clockwise.
            speed: The angular speed to rotate, in radians/second.
        """
        # goal - current > 0 ==> fine. if goal - current < 0 ==> (|goal - current| // 180) * 360 + (goal - current)
        # goal = current, current = goal
        # TODO: rospy.sleep until the base has received at least one message on /odom
        # TODO: record start position, use Python's copy.deepcopy
        rospy.sleep(1)
        start = copy.deepcopy(self.current_pose)
        # TODO: What will you do if angular_distance is greater than 2*pi or less than -2*pi?
        rate = rospy.Rate(10)
        # TODO: CONDITION should check if the robot has rotated the desired amount
        # TODO: Be sure to handle the case where the desired amount is negative!
        curr_rotation = start.theta
        clockwise = angular_distance
        angular_distance = angular_distance % (2 * math.pi)
        clockwise = np.sign(clockwise) * angular_distance
        # exit()
        dest = (angular_distance + curr_rotation)  % (2 * math.pi)

        curr_trav_dist = self.calculate_angular_dist(dest, self.current_pose.theta, clockwise)
        min_val = float('inf')
        while True:
            # TODO: you will probably need to do some math in this loop to check the CONDITION
            # print("Hello")
            direction = -1 if clockwise < 0 else 1
            self.move(0, direction * speed)
            curr_trav_dist = self.calculate_angular_dist(dest, self.current_pose.theta, clockwise)
            print(curr_trav_dist)
            min_val = min(min_val, curr_trav_dist)

            if curr_trav_dist > min_val and min_val < 0.05:
                # self.move(0, -direction * speed * 0.4)
                break
            # curr_rotation = self.current_pose.theta
            rate.sleep()
        
    def calculate_angular_dist(self, goal, curr, angular_distance):

        # if goal > curr:
        #     if angular_distance > 0:
        #         return goal - curr
        #     else:
        #         return 2*math.pi - (goal - curr)
        # else: # curr > goal
        #     if angular_distance > 0:
        #         return 2*math.pi + (goal - curr)
        #     else:
        #         return curr - goal

        return (np.sign(angular_distance) * (goal - curr)) % (2 * math.pi)

    def move(self, linear_speed, angular_speed):
        """Moves the base instantaneously at given linear and angular speeds.

        "Instantaneously" means that this method must be called continuously in
        a loop for the robot to move.

        Args:
            linear_speed: The forward/backward speed, in meters/second. A
                positive value means the robot should move forward.
            angular_speed: The rotation speed, in radians/second. A positive
                value means the robot should rotate clockwise.
        """
        msg = Twist()
        msg.angular.z = angular_speed
        msg.linear.x = linear_speed
        self.pub.publish(msg)

    def stop(self):
        """Stops the mobile base from moving.
        """
        self.move(0, 0)
