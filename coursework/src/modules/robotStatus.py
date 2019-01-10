#!/usr/bin/env python

import rospy
import numpy as np
from goToPoint import GoToPose
from geometry_msgs.msg import Twist
from math import radians
from kobuki_msgs.msg import BumperEvent
from modules import Tracker
from modules import CluedoClassifier

class RobotStatus:

    ####
    ## Stores the coordinates for the entrance and centre of the  room and general
    ## configurations for the robot
    ####
    def __init__(self):
        self.centreXcoordinate = 6.38
        self.centreYcoordinate = -0.1
        self.entranceXcoordinate = 5.38
        self.entranceYcoordinate = -0.1
        self.goToPose = GoToPose()
        self.tracker = Tracker()
        self.cluedoClassifier = CluedoClassifier()
    	self.movement_pub = rospy.Publisher('mobile_base/commands/velocity', Twist, queue_size=10)
    	self.rate = rospy.Rate(10) #10hz
        self.desired_velocity = Twist()
        rospy.Subscriber("mobile_base/events/bumper", BumperEvent, self.processBump)

    ####
    ## Ths function is used to go to the middle of the given environment, using the centreXcoordinate
    ## and centreYcoordinate.
    ####
    def goToMiddle(self):
        rospy.loginfo("start of go to middle function")
        success = self.goToPose.goToPosition(self.centreXcoordinate, self.centreYcoordinate, 0.00)
        if success:
            rospy.loginfo("The robot made it to the middle")
            self.rotate(360)
        else:
            rospy.loginfo("The Robot couldn't get this this position")
            self.goToPose.shutdown()

    ####
    ## Ths method is used to go to the entrance of the given environment, using the centreXcoordinate
    ## and centreYcoordinate.
    ####
    def goToEntrance(self):
        success = self.goToPose.goToPosition(self.entranceXcoordinate, self.entranceYcoordinate, 0.00)
        if success:
            rospy.loginfo("RobotStatus class made it to the entrance")
        else:
            rospy.loginfo("The Robot couldn't get this this position")
            self.goToPose.shutdown()

    ####
    ## This method stops the movement of the robot, it's used when analysing
    ## images
    ####
    def stopMovement(self):
        self.desired_velocity.linear.x = 0
        self.desired_velocity.angular.z = 0
        self.movement_pub.publish(self.desired_velocity)

    ####
    ## This method rotates the robot by the given angleValue specified when
    ## calling the method
    ####
    def rotate(self, angleValue):
        end_angle = radians(angleValue)
        self.desired_velocity.linear.x = 0
        self.desired_velocity.angular.z = 0.3
        t0 = rospy.Time.now().to_sec()
        current_angle = 0

        while current_angle <= end_angle:
            self.movement_pub.publish(self.desired_velocity)
            t1 = rospy.Time.now().to_sec()
            current_angle = 0.5*(t1-t0)

        self.desired_velocity.angular.z = 0
        self.movement_pub.publish(self.desired_velocity)

    ####
    ## Processes bumps should it encounter one en route to the entrance or
    ## the centre of the given environment
    ####
    def processBump(self, data):
    	if (data.state == BumperEvent.PRESSED):
            rospy.loginfo('collision... correcting position')
            self.desired_velocity.linear.x = 0
            self.desired_velocity.angular.z = 0
            self.movement_pub.publish(self.desired_velocity)
            rospy.sleep(1)
            rospy.loginfo('reversing...')
            for i in range(0,30):
                self.desired_velocity.linear.x = 0.2
                self.movement_pub.publish(self.desired_velocity)
            rospy.loginfo('recovered moving on')
