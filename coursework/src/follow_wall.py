#!/usr/bin/env python
import roslib
import rospy
import numpy as np
import math
import time
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32

class FollowWall:
    def __init__(self):
        # subscribe to laserscan topic
        rospy.Subscriber('/scan', LaserScan, self.laserscan_callback)
        self.velocityPublish = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size = 10)

        self.velocity = Twist()
        self.laserscan = None

        self.wall_dist = 0.13
        self.max_speed = 0.3
        self.direction = 1
        self.p = 1
        self.d = 0.5
        self.angle = 1

        self.e = 0
        # Angle, at which was measured the shortest distance
        self.angle_minDist = 0
        self.dist_front = 0
        self.diff_e = 0

    def laserscan_callback(self, scan_data):
        # append non nan values to a new list
        laserValues = []
        for i in xrange(0, len(scan_data.ranges)):
            if not np.isnan(scan_data.ranges[i]):
                laserValues.append(scan_data.ranges[i])

        size = len(laserValues)
        min_index = size * (self.direction + 1)/4
        max_index = size * (self.direction + 3)/4

        # find minimum
        for i in range(min_index, max_index):
            if (laserValues[i] < laserValues[min_index] and laserValues[i] > 0.01):
                min_index = i

        # Calculation of angles from indexes and storing data to class variables
        self.angle_minDist = (min_index - size/2) * scan_data.angle_increment
        dist_min = laserValues[min_index]
        self.dist_front = laserValues[size/2]
        self.diff_e = (dist_min - self.wall_dist) - self.e
        self.e = dist_min - self.wall_dist

        self.movement()

    def movement(self):
        # PD controller
        self.velocity.angular.z = self.direction*(self.p*self.e+self.d*self.diff_e) + self.angle*(self.angle_minDist-math.pi*self.direction/2)
        if (self.dist_front < self.wall_dist):
            print('dist_front < wall_dist')
            self.velocity.linear.x = 0
        elif (self.dist_front < self.wall_dist * 2):
            print('dist_front < wall_dist * 2')
            self.velocity.linear.x = 0.5 * self.max_speed
        elif (abs(self.angle_minDist) > 1.75):
            print('angle_minDist > 1.75')
            self.velocity.linear.x = 0.4 * self.max_speed
        else:
            print('else')
            self.velocity.linear.x = self.max_speed

        self.velocityPublish.publish(self.velocity)

if __name__ == '__main__':
    try:
        rospy.init_node('wallFollower', anonymous=True)
        wallFollower = FollowWall()
        # don't exit until stopped
        rospy.spin()
    except rospy.ROSInterruptException:
		pass
