#!/usr/bin/env python

import rospy
import numpy as np
from goToPoint import GoToPose
from geometry_msgs.msg import Twist
from math import radians
from modules import Tracker

class RobotStatus:

    def __init__(self):
        self.run = True
        self.centreXcoordinate = -4.8
        self.centreYcoordinate = -0.6
        self.entranceXcoordinate = 0.00
        self.entranceYcoordinate = 0.00
        self.goToPose = GoToPose()
        self.tracker = Tracker()

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
            rospy.loginfo("RobotStatus class made it to the middle")
        else:
            rospy.loginfo("The Robot couldn't get this this position")

    def rotate(self, angleValue):
    	movement_pub = rospy.Publisher('mobile_base/commands/velocity', Twist, queue_size=10)
    	rate = rospy.Rate(10) #10hz
        end_angle = radians(angleValue)
    	desired_velocity = Twist()
        desired_velocity.linear.x = 0
        desired_velocity.angular.z = 2
        t0 = rospy.Time.now().to_sec()
        current_angle = 0

        while current_angle <= end_angle:
            movement_pub.publish(desired_velocity)
            t1 = rospy.Time.now().to_sec()
            current_angle = 2*(t1-t0)
        print(self.tracker.postercounter)
        print(self.tracker.arlist[0])


        desired_velocity.angular.z = 0
        movement_pub.publish(desired_velocity)


    def processBump(data):
    	if (data.state == BumperEvent.PRESSED):
    		global bump
    		bump = True

if __name__ == "__main__":
    rospy.init_node('robotStatus_main', anonymous=True)
    robotStatus = RobotStatus()
    robotStatus.goToMiddle()

# -4.8, 3.75
# -5, - 0.6
