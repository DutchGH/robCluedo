#!/usr/bin/env python

import rospy
import numpy as np
from goToPoint import GoToPose
from geometry_msgs.msg import Twist
from math import radians

class RobotStatus:

    def __init__(self):
        self.run = True
        self.centreXcoordinate = -4.8
        self.centreYcoordinate = 3.75
        self.entranceXcoordinate = 0.00
        self.entranceYcoordinate = 0.00
        self.goToPose = GoToPose()

    def goToMiddle(self):
        success = self.goToPose.goToPosition(self.centreXcoordinate, self.centreYcoordinate, 0.00)
        if success:
            self.rotate(360)
            rospy.loginfo("RobotStatus class made it to the middle")
        else:
            rospy.loginfo("The Robot couldn't get this this position")


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

        while current_angle < end_angle:
            movement_pub.publish(desired_velocity)
            t1 = rospy.Time.now().to_sec()
            current_angle = 2*(t1-t0)

        desired_velocity.angular.z = 0
        movement_pub.publish(desired_velocity)


    def processBump(data):
    	if (data.state == BumperEvent.PRESSED):
    		global bump
    		bump = True


# -4.8, 3.75

if __name__ == '__main__':
    rospy.init_node('robotStatus', anonymous=True)
    robotStatus = RobotStatus()
    robotStatus.goToMiddle()
