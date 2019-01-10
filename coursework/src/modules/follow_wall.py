#!/usr/bin/env python
import roslib
import rospy
import numpy as np
import math
import time
import tf
from math import radians
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32
from kobuki_msgs.msg import BumperEvent


class FollowWall:
    def __init__(self):
        # subscribe to laserscan topic
        self.velocityPublish = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size = 10)
        self.tf_listener = tf.TransformListener()
        rospy.Subscriber("mobile_base/events/bumper", BumperEvent, self.processBump)
        self.velocity = Twist()

        self.counter = 0
        self.turnAround = False
        # minimum distance from wall
        self.minDist_wall = 0.25
        # distance at middle of laserscan - in front of robot
        self.distAhead = 0
        self.speed = 0.2
        self.maxAngularVel = 1

        # angle against minimum distance point on map
        self.angle_minDist = 0
        # error of distance to closest object - distance from wall
        self.error_value = 0
        # difference from previous error
        self.diff_PrevError = 0

    def start(self, entranceXcoord, entranceYcoord):
        self.entranceXcoord = entranceXcoord
        self.entranceYcoord = entranceYcoord
        self.laserscanSubs = rospy.Subscriber('/scan', LaserScan, self.laserscan_callback)

    def stop(self):
        self.laserscanSubs.unregister()

    def checkEntrance(self):
        t = self.tf_listener.getLatestCommonTime("/base_link", "/map")
        position, quaternion = self.tf_listener.lookupTransform("/base_link", "/map", t)
        # print(position)
        print('x coordinate ' + str(abs(position[1] - self.entranceXcoord - 0.25)))
        print('y coordinate ' + str(abs(position[0] - self.entranceYcoord - 0.25)))
        if abs(position[1] - self.entranceXcoord - 0.25) < 0.4 and abs(position[0] - self.entranceYcoord -0.25)< 0.4:
            # print('inside checkEntrance if')
            return True
        return False


    def laserscan_callback(self, scan_data):
        self.counter += 1
        # append non nan values to a new list
        laserValues = []
        for i in xrange(0, len(scan_data.ranges)):
            if not np.isnan(scan_data.ranges[i]):
                laserValues.append(scan_data.ranges[i])

        size = len(laserValues)
        if (size == 0):
            # laser scan not picking up anything, too close to object
            # stop scanning, reverse and rotate
            self.stop()
            self.stopMovement()
            self.reverseAndRotate()
            # start scanning again
            self.start(self.entranceXcoord, self.entranceYcoord)
        else :
            minIndex = int(size/2)
            self.distAhead = laserValues[minIndex]

            # find minimum index value
            for i in range(minIndex, size):
                if (laserValues[i] < laserValues[minIndex] and laserValues[i] > 0.01):
                    minIndex = i

            self.angle_minDist = (minIndex - size/2) * scan_data.angle_increment
            # laser scan value at minimum index
            minIndex_value = laserValues[minIndex]
            self.diff_PrevError = (minIndex_value - self.minDist_wall) - self.error_value
            self.error_value = minIndex_value - self.minDist_wall

            self.movement()

    def movement(self):
        velocity = self.error_value + self.diff_PrevError + self.angle_minDist - math.pi * 0.5
        # avoid looping if posters not detected
        lapComplete = self.checkEntrance()
        # print(self.counter)
        if lapComplete and self.counter > 80 and self.turnAround == False:
            print('inside turn around if')
            self.stop()
            self.stopMovement()
            self.rotate(270)
            rospy.sleep(5)
            self.turnAround = True
            self.start(self.entranceXcoord, self.entranceYcoord)
        # manage navigation
        if (velocity > 1):
            velocity = self.maxAngularVel
        self.velocity.angular.z = velocity
        if (self.distAhead < self.minDist_wall):
            # too close to wall ahead, stop
            self.velocity.linear.x = 0
        elif (self.distAhead < self.minDist_wall * 2):
            # geeting close to wall, slow down
            self.velocity.linear.x = 0.5 * self.speed
        else:
            self.velocity.linear.x = self.speed

        self.velocityPublish.publish(self.velocity)

    def processBump(self, data):
        # if robot bumps into an obstacle, stop moving and reverse
    	if (data.state == BumperEvent.PRESSED):
            rospy.loginfo('hit something... correcting')
            self.velocity.linear.x = 0
            self.velocity.angular.z = 0
            self.velocityPublish.publish(self.velocity)
            rospy.sleep(1)
            rospy.loginfo('reversing...')
            for i in range(0,30):
                self.velocity.linear.x = -0.2
                self.velocityPublish.publish(self.velocity)
            rospy.loginfo('recovered moving on')

    def stopMovement(self):
        self.velocity.linear.x = 0
        self.velocity.angular.z = 0
        self.velocityPublish.publish(self.velocity)

    def reverseAndRotate(self):
        # too close to object, reverse and rotate
        self.velocity.linear.x = -0.2
        self.velocityPublish.publish(self.velocity)
        rospy.sleep(0.5)
        self.stopMovement()
        self.rotate(140)
        rospy.sleep(1)

    def rotate(self, angle):
        self.velocity.linear.x = 0
        self.velocity.angular.z = -radians(angle)
        self.velocityPublish.publish(self.velocity)
