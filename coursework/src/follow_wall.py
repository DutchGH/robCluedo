#!/usr/bin/env python

import roslib
import rospy
import numpy as np
import math
from math import radians
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32

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
        self.wallDist = 0.9
        self.angle = 1.57284
        self.centre = Float32()
        self.left = Float32()
        self.right = Float32()
        self.left_last = Float32()
        self.right_last = Float32()

    def laserscan_callback(self, laser_data):
        self.centre.data= laser_data.ranges[320]
        rospy.loginfo("centre %f", laser_data.ranges[320])
        self.left.data= laser_data.ranges[639]
        rospy.loginfo("left %f", laser_data.ranges[639])
        self.right.data= laser_data.ranges[0]
        rospy.loginfo("right %f", laser_data.ranges[0])
        self.laserscan = laser_data

    def start(self):
        rospy.loginfo('Start Following Wall')

        while not rospy.is_shutdown():
            if (self.laserscan):
                # minLaserValue = min(min(self.laserscan.ranges[170:190], self.laserscan.ranges[150:170]))
                # self.findWall(minLaserValue)
                self.followWall()

            self.rate.sleep()
            # rospy.spin()

    def findClosestWall():
        print('Closest Wall')
        # go to it and followWall()
        # if get too far away from wall findClosestWall()

    def followWall(self):
        if (self.laserscan):
            print('START')
            if (self.centre.data > 1):
                # move forward
                print('centre > 1')
                self.moveForward()
                if (self.left.data < 0.8):
                    # close to left object, turn right
                    print('left < 0.8')
                    self.rotateRight()
                elif (self.right.data < 0.8):
                    # close to right object, turn left
                    print('right < 0.8')
                    self.rotateLeft()
            else :
                print(' else ')
                if (self.left.data > 0.8):
                    print('left > 0.8')
                    self.rotateRight()
                elif (self.right.data > 0.8):
                    print('right > 0.8')
                    self.rotateLeft()
                elif (self.left.data > self.right.data):
                    print('left > right')
                    self.rotateRight()
                else :
                    print ('right > left')
                    self.rotateLeft()

        self.rate.sleep()

    def stop(self):
        self.velocity.linear.x = 0.0
        self.velocity.angular.z = 0
        self.velocityPublish.publish(self.velocity)

    def moveForward(self):
        self.velocity.linear.x = 0.2
        self.velocity.angular.z = 0
        self.velocityPublish.publish(self.velocity)

    def rotateRight(self):
        self.velocity.linear.x = 0
        self.velocity.angular.z = -self.angle
        self.velocityPublish.publish(self.velocity)

    def rotateLeft(self):
        self.velocity.linear.x = 0
        self.velocity.angular.z = self.angle
        self.velocityPublish.publish(self.velocity)

if __name__ == '__main__':
    try:
        wallFollower = FollowWall()
        wallFollower.start()
    except rospy.ROSInterruptException:
		pass
