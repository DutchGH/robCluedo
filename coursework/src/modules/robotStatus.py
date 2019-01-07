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

    def __init__(self):
        self.run = True
        # lab5 world
        # self.centreXcoordinate = 0.04
        # self.centreYcoordinate = 3
        # self.entranceXcoordinate = 0.04
        # self.entranceYcoordinate = 1
        # demo world
        self.centreXcoordinate = 5.38
        self.centreYcoordinate = -0.1
        self.entranceXcoordinate = 5.38
        self.entranceYcoordinate = -0.1
        self.goToPose = GoToPose()
        self.tracker = Tracker()
        self.cluedoClassifier = CluedoClassifier()
    	self.movement_pub = rospy.Publisher('mobile_base/commands/velocity', Twist, queue_size=10)
    	self.rate = rospy.Rate(10) #10hz
        self.desired_velocity = Twist()
        # rospy.Subscriber("mobile_base/events/bumper", BumperEvent, self.processBump)

    def goToMiddle(self):
        rospy.loginfo("start of go to middle function")
        success = self.goToPose.goToPosition(self.centreXcoordinate, self.centreYcoordinate, 0.00)
        if success:
            rospy.loginfo("The robot made it to the middle")
            self.rotate(360)
        else:
            rospy.loginfo("The Robot couldn't get this this position")
            self.goToPose.shutdown()

    def goToEntrance(self):
        success = self.goToPose.goToPosition(self.entranceXcoordinate, self.entranceYcoordinate, 0.00)
        if success:
            rospy.loginfo("RobotStatus class made it to the enterance")
        else:
            rospy.loginfo("The Robot couldn't get this this position")
            self.goToPose.shutdown()

    def stopMovement(self):
        self.desired_velocity.linear.x = 0
        self.desired_velocity.angular.z = 0
        self.movement_pub.publish(self.desired_velocity)

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


    def processBump(self, data):
    	if (self.data.state == BumperEvent.PRESSED):
            rospy.loginfo('hit something... correcting')
            self.desired_velocity.linear.x = 0
            self.desired_velocity.angular.z = 0
            self.movement_pub.publish(self.desired_velocity)
            rospy.sleep(1)
            rospy.loginfo('reversing...')
            for i in range(0,30):
                self.desired_velocity.linear.x = 0.2
                self.movement_pub.publish(self.desired_velocity)
            rosp.loginfo('recovered moving on')


    def produceTxtFile(self):
        file = open('ImageInformation.txt', 'w')
        for i in range(1,3):
            file.write('Image %s information', i)
            file.write('image name:')
            # name of image here Jake
            file.write('image location:')
            file.write(self.tracker.arlist[i])
            file.write(self.tracker.quatList[i])
            file.close()
