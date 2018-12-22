#!/usr/bin/env python
import roslib
import rospy
import numpy as np
import math
import time
from math import radians
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32

class FollowWall:
    def __init__(self):
        # subscribe to laserscan topic
        self.velocityPublish = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size = 10)

        self.velocity = Twist()

        self.wall_dist = 0.13
        self.speed = 0.2
        self.d = 0.4

        self.e = 0
        # Angle, at which the shortest distance was measured
        self.angle_minDist = 0
        self.dist_front = 0
        self.diff_e = 0

    def start(self):
        self.laserscanSubs = rospy.Subscriber('/scan', LaserScan, self.laserscan_callback)

    def stop(self):
        self.laserscanSubs.unregister()

    def laserscan_callback(self, scan_data):
        # append non nan values to a new list
        laserValues = []
        for i in xrange(0, len(scan_data.ranges)):
            if not np.isnan(scan_data.ranges[i]):
                laserValues.append(scan_data.ranges[i])

        size = len(laserValues)
        min_index = int(size/2)
        max_index = size

        # find minimum
        for i in range(min_index, max_index):
            if (laserValues[i] < laserValues[min_index] and laserValues[i] > 0.2):
                min_index = i

        self.angle_minDist = (min_index - (size/2)) * scan_data.angle_increment
        dist_min = laserValues[min_index]
        self.dist_front = laserValues[int(size/2)]
        self.diff_e = (dist_min - self.wall_dist) - self.e
        self.e = dist_min - self.wall_dist

        self.movement()

    def movement(self):
        # PD controller
        self.velocity.angular.z = (self.e + self.d * self.diff_e) + (self.angle_minDist - math.pi * 0.5)
        if (self.dist_front < self.wall_dist):
            self.velocity.linear.x = 0
        elif (self.dist_front < self.wall_dist * 2):
            self.velocity.linear.x = 0.5 * self.speed
        elif (abs(self.angle_minDist) > 1.75):
            self.velocity.linear.x = 0.4 * self.speed
        else:
            self.velocity.linear.x = self.speed

        self.velocityPublish.publish(self.velocity)

    def rotate(self):
        end_angle = radians(360)
        self.velocity.linear.x = 0
        self.velocity.angular.z = 0.3
        t0 = rospy.Time.now().to_sec()
        current_angle = 0

        while current_angle <= end_angle:
            self.velocityPublish.publish(self.velocity)
            t1 = rospy.Time.now().to_sec()
            current_angle = 0.5*(t1-t0)

        self.velocity.angular.z = 0
        self.velocityPublish.publish(self.velocity)
