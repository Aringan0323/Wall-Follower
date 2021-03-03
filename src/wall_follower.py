#!/usr/bin/env python

import rospy
from os import system
import sys
import math
import tf
import time
import random
import numpy as np
from numpy import inf
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class follower:

    def __init__(self, rate=10):

        rospy.init_node('robot_controller')

        self.scan_sub = rospy.Subscriber('scan', LaserScan, self.scan_cb)
        self.odom_sub = rospy.Subscriber('odom', Odometry, self.odom_cb)
        self.move = rospy.Publisher('cmd_vel', Twist, queue_size=10)

        self.rate = rospy.Rate(rate)

        self.twist = Twist()

        self.curr_x = 0
        self.curr_z = 0


    def scan_cb(self, msg):
        self.ranges = np.array(msg.ranges)

    def clean_ranges(self):
        clean_ranges = self.ranges
        clean_ranges[clean_ranges == inf] = 4
        clean_ranges[clean_ranges <= 0] = 0
        return clean_ranges

    def odom_cb(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.xq = msg.pose.pose.orientation.x
        self.yq = msg.pose.pose.orientation.y
        self.zq = msg.pose.pose.orientation.z
        self.wq = msg.pose.pose.orientation.w

    def get_rotation(self):
        t3 = 2.0 * (self.wq * self.zq + self.xq * self.yq)
        t4 = 1.0 - 2.0 * (self.yq * self.yq + self.zq * self.zq)
        rot = math.atan2(t3, t4)
        return rot


    def nearest_angle(self):
        return np.argmin(self.clean_ranges())

    def farthest_angle(self):
        return np.argmax(self.clean_ranges())

    def update_twist(self):
        self.twist.linear.x = self.curr_x
        self.twist.angular.z = self.curr_z


    def PID(self, )


    def correct_vel_linear(self, diff, max_vel):
        return -((max_vel*5)/(2**(20*diff + 2.36))) + max_vel


    def correct_vel_rotation(self, diff, max_vel):
        return -((max_vel*5)/(2**(2*diff + 2.4))) + max_vel

    def diff(self, a, b):
        return abs(b-a)

    def get_direction(self, curr, target):
        return np.sign((curr - target) * (abs(curr-target) - math.pi))


    def rotate_towards_angle(self, angle):
        curr = self.get_rotation()
        diff = self.diff(curr,angle)
        direction = self.get_direction(curr, angle)
        while (diff > 0.005):
            curr = self.get_rotation()
            diff = self.diff(curr, angle)
            self.curr_z = self.correct_vel_rotation(diff, math.pi/3) * direction
            self.update_twist()
            self.move.publish(self.twist)
        self.curr_z = 0
        self.update_twist()
        self.move.publish(self.twist)


    def move_to_wall(self):
        angle_to_wall = self.nearest_angle()
        self.rotate_towards_angle()
        self.
    
    def follow_wall(self, break_cb, vel_cb):
        while not break_cb:
            self.curr_x, self.curr_z = vel_cb()
            self.update_twist()
            self.move.publish(self.twist)
        self.curr_x, self.curr_z = (0,0)
        self.update_twist()
        self.move.publish(self.twist)



    def 

    def break_cb(self):


    