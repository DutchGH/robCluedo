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
from kobuki_msgs.msg import BumperEvent

class FollowWall:
    def __init__(self):
        # subscribe to laserscan topic
        # rospy.Subscriber('/scan', LaserScan, self.laserscan_callback)
        self.velocityPublish = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size = 10)
        # rospy.Subscriber("mobile_base/events/bumper", BumperEvent, self.processBump)
        self.velocity = Twist()

        self.wall_dist = 0.13
        self.max_speed = 0.2
        self.direction = 1
        self.p = 1
        self.d = 0.4
        self.angle = 1
        self.turnAround = False
        self.counter = 0

        self.e = 0
        # Angle, at which was measured the shortest distance
        self.angle_minDist = 0
        self.dist_front = 0
        self.diff_e = 0

    def start(self, entranceXcoord, entranceYcoord):
        self.entranceXcoord = entranceXcoord
        self.entranceYcoord = entranceYcoord
        self.startCntr = True
        self.laserscanSubs = rospy.Subscriber('/scan', LaserScan, self.laserscan_callback)

    def stop(self):
        self.startCntr = False
        self.laserscanSubs.unregister()

    def checkEnterance(self):
        if self.tf.frameExists("/base_link") and self.tf.frameExists("/map"):
            t = self.tf.getLatestCommonTime("/base_link", "/map")
            position, quaternion = self.tf.lookupTransform("/base_link", "/map", t)
            if abs(position.x - self.entranceXcoord) < 0.04 and abs(position.y - self.entranceYcoord)< 0.04:
                return True

    def laserscan_callback(self, scan_data):
        self.counter += 1
        self.updateValues()
        # # append non nan values to a new list
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

    def updateValues(self):
        self.wall_dist = 0.13
        self.max_speed = 0.2
        self.direction = 1
        self.p = 1
        self.d = 0.5
        self.angle = 1
        self.e = 0
        # Angle, at which was measured the shortest distance
        self.angle_minDist = 0
        self.dist_front = 0
        self.diff_e = 0

    def movement(self):
        # PD controller
        lapComplete = self.checkEnterance
        if lapComplete and self.counter > 500 and self.turnAround == False:
            self.stop()
            self.rotate(180)
            self.turnAround = True
            self.start(self.entranceXcoord, self.entranceYcoord)
        self.velocity.angular.z = self.direction*(self.p*self.e+self.d*self.diff_e) + self.angle*(self.angle_minDist-math.pi*self.direction/2)
        if (self.dist_front < self.wall_dist):
            self.velocity.linear.x = 0
        elif (self.dist_front < self.wall_dist * 2):
            self.velocity.linear.x = 0.5 * self.max_speed
        elif (abs(self.angle_minDist) > 1.75):
            self.velocity.linear.x = 0.4 * self.max_speed
        else:
            self.velocity.linear.x = self.max_speed
        self.velocityPublish.publish(self.velocity)

    # def processBump(self, data):
    # 	if (data.state == BumperEvent.PRESSED):
    #         rospy.loginfo('hit something... correcting')
    #         self.velocity.linear.x = 0
    #         self.velocity.angular.z = 0
    #         self.velocityPublish.publish(self.velocity)
    #         rospy.sleep(1)
    #         rospy.loginfo('reversing...')
    #         for i in range(0,30):
    #             self.velocity.linear.x = -0.2
    #             self.velocityPublish.publish(self.velocity)
    #         rospy.loginfo('recovered moving on')

    def rotate(self, angleValue):
        end_angle = radians(angleValue)
        self.velocity.linear.x = 0
        self.velocity.angular.z = 0.5
        t0 = rospy.Time.now().to_sec()
        current_angle = 0

        while current_angle <= end_angle:
            self.velocityPublish.publish(self.velocity)
            t1 = rospy.Time.now().to_sec()
            current_angle = 0.5*(t1-t0)

        self.velocity.angular.z = 0
        self.velocityPublish.publish(self.velocity)
