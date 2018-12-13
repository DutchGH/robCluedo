#!/usr/bin/env python

import rospy
import numpy as np
from goToPoint import GoToPose

class RobotStatus:

    def __init__(self):
        self.run = True
        self.centreXcoordinate = -4.8
        self.centreYcoordinate = 3.75
        self.narrowXcoordinate = 0.00
        self.narrowYcoordinate = 0.00
        self.goToPose = GoToPose()

    def goToMiddle(self):
        success = self.goToPose.goToPosition(self.centreXcoordinate, self.centreYcoordinate, 0.00)
        if success:
            rospy.loginfo("RobotStatus class made it to the middle")
        else:
            rospy.loginfo("The Robot couldn't get this this position")

    # def goToEnterance:
    #     self.goToPoint


# -4.8, 3.75

if __name__ == '__main__':
    rospy.init_node('robotStatus', anonymous=True)
    robotStatus = RobotStatus()
    robotStatus.goToMiddle()
