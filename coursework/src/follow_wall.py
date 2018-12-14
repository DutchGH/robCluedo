#!/usr/bin/env python

import roslib
import rospy
import numpy as np
import math
from math import radians
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class FollowWall:
    def __init__(self):
        rospy.init_node('WallFollowerNode')

        # subscribe to laserscan topic
        rospy.Subscriber('/scan', LaserScan, self.laserscan_callback)
        self.velocityPublish = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size = 10)

        self.rate = rospy.Rate(10)
        self.velocity = Twist()
        self.laserscan = None

        self.spinSpeed = 0.1
        self.wallDist = 0.8
        self.angle = 0.9

    def laserscan_callback(self, laser_msg):
        self.laserscan = laser_msg

    def start(self):
        rospy.loginfo('Start Following Wall')

        while not rospy.is_shutdown():
            if (self.laserscan):
                minLaserValue = min(min(self.laserscan.ranges[170:190], self.laserscan.ranges[150:170]))
                self.velocity.linear.x = 0.2
                self.velocity.angular.z = 0.0
                self.findWall(minLaserValue)

            self.velocityPublish.publish(self.velocity)

            self.rate.sleep()
            # rospy.spin()

    def findWall(self, minLaserValue):
        rospy.loginfo('Find Wall')
        print(minLaserValue)
        print(' -- ')
        print(self.wallDist)
        if(minLaserValue < self.wallDist):
            self.rotate(self.angle, self.spinSpeed)
        else:
            print('minLaserValue > wallDist')

    def rotate(self, angle, speed):
        self.velocity.linear.x = 0.0
        self.velocity.angular.z = radians(45)

        cur_angle = 0
        cur_time = rospy.Time.now().to_sec()

        self.velocityPublish.publish(self.velocity)
        # self.stop()

    def stop(self):
        self.velocity.linear.x = 0.0
        self.velocity.angular.z = 0
        self.velocityPublish.publish(self.velocity)

if __name__ == '__main__':
    try:
        wallFollower = FollowWall()
        wallFollower.start();
    except rospy.ROSInterruptException:
		pass
