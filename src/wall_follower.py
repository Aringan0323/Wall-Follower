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

        self.ranges = np.zeros(360)


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
        return np.argmin(self.clean_ranges()) * math.pi/180

    def farthest_angle(self):
        return np.argmax(self.clean_ranges()) * math.pi/180

    def nearest_dist(self):
        return np.amin(self.clean_ranges())

    def farthest_dist(self):
        return np.amax(self.clean_ranges())
    

    def update_twist(self, x, z):
        self.curr_x = x
        self.curr_z = z
        self.twist.linear.x = x
        self.twist.angular.z = z
        self.move.publish(self.twist)

    def correct_vel_linear(self, diff, max_vel):
        return -((max_vel*5)/(2**(20*diff + 2.36))) + max_vel


    def correct_vel_rotation(self, diff, max_vel):
        return -((max_vel*5)/(2**(2*diff + 2.4))) + max_vel

    def diff(self, a, b):
        return abs(b-a)

    def dist(self, a, b):
        return np.sqrt((a[0]-b[0])**2 + (a[1]-b[1])**2)

    def get_direction(self, curr, target):
        return np.sign((curr - target) * (abs(curr-target) - math.pi))


    def rotate_towards_angle(self, angle):
        curr = self.get_rotation()
        if curr > math.pi:
            curr = curr - math.pi*2
        diff = self.diff(curr,angle)
        direction = self.get_direction(curr, angle)
        while (diff > 0.005) and not rospy.is_shutdown():
            curr = self.get_rotation()
            if curr > math.pi:
                curr = curr - math.pi*2
            diff = self.diff(curr, angle)
            z = self.correct_vel_rotation(diff, math.pi/3) * direction
            self.update_twist(self.curr_x, z)
            self.rate.sleep()
        self.update_twist(self.curr_x, 0)


    def move_to_wall(self):
        self.update_twist(0,0)
        angle_to_wall = self.nearest_angle()
        if angle_to_wall > math.pi:
            angle_to_wall = angle_to_wall - math.pi*2
        self.rotate_towards_angle(angle_to_wall)
        while self.nearest_dist() > 0.5:
            x = self.correct_vel_linear(self.nearest_dist()-0.49, 0.2)
            self.update_twist(x, 0)
        self.update_twist(0,0)
    
    def follow_wall(self, kp, kd, kp2):
        self.tn_prev = rospy.Time.now().to_sec()
        self.dmin_prev = self.nearest_dist()
        while self.follow_wall_break_cb() and not rospy.is_shutdown():
            z = self.PID(kp, kd, kp2)
            self.update_twist(0.2, z)
            self.rate.sleep()
        self.update_twist(0,0)



    def PID(self, kp, kd, kp2):
        tn = rospy.Time.now().to_sec()
        dmin = self.nearest_dist()
        dwall = 0.5
        amin = self.nearest_angle()
        if amin <= math.pi:
            di = 1
        else:
            di = -1
        
        wall_err = dmin - dwall
        wall_err_prev = self.dmin_prev - dwall
        PDct = kp * wall_err + kd * ( (wall_err-wall_err_prev) / (tn-self.tn_prev+1e-10) )

        a_err = amin - (math.pi/2)*di
        Pct = kp2*a_err

        final_angular_velocity = PDct + Pct

        self.tn_prev = tn
        self.dmin_prev = dmin

        return final_angular_velocity


    def follow_wall_break_cb(self):
        if rospy.is_shutdown():
            return False 
        else:
            return True

    
    def find_wall_break_cb(self):
        if self.nearest_dist >= 0.5:
            return True
        else:
            return False



f = follower()

time.sleep(0.5)



f.move_to_wall()

f.follow_wall(0.5,0.5,0.5)

        


    