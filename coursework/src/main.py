#!/usr/bin/env python

import rospy
import sys
import cv2
import os
import tf
import random
import math

from sensor_msgs.msg import Image
from sensor_msgs.msg import LaserScan
from ar_track_alvar_msgs.msg import AlvarMarkers
from kobuki_msgs.msg import BumperEvent
from std_msgs.msg import String, Bool
from modules import robotStatus
from modules import Tracker
from modules import cluedoClassifier


class robCluedo:
    def __init__(self, pub, rate):
        self.image_sub = rospy.Subscriber('CluARFound', Bool, self.callback)
        self.murderer = None
        self.murderWeapon = None
        print("Up and Running")

    def callback(self, data):
        print(data.data)

def publisher(name, type):
	pub = rospy.Publisher(name, type, queue_size=10)
	rate = rospy.Rate(10) #10hz

	return pub, rate

def main():
    rospy.init_node('cluedo_main', anonymous=True)
    pub, rate = publisher("CluedoMain", Bool)
    cI = robCluedo(pub, rate)
    running = True

    while running:
        try:
            robotRunning = robotStatus.RobotStatus()
            robotRunning.goToMiddle()

            if robotRunning.tracker.postercounter == 2:
                for i in range(0,2):
                    gotToDest = robotRunning.tracker.position(i)
                    if goToDest:
                        print('000000')
                        robotRunning.cluedoClassifier.imageAnalysis()
                        print('finished image analysis')
                        ########## Jake #########
                        #### infront of the poster mate insert your code here
                        ########## Jake #########
                        ###write file
                        ##end program
                        running = False
            if robotRunning.tracker.postercounter <= 1:
                    print('00000')
                    if robotRunning.tracker.postercounter == 1:
                        print('before moving to position')
                        goToDest = robotRunning.tracker.position(0)
                        if goToDest:
                            cluedoClassifier.main()
                            ##### Jake ######
                            #### check images if found
                    ### run wall following alogrithms
                    ###### Emily ######
                    ##### Insert wall following here
                    ##### Jake ######
                    #### check images if found
                    ### running = False
            else:
                ###commit suicide 
            rospy.spin()
        except KeyboardInterrupt:
            print("Shutting down")

if __name__ == "__main__":
    main()
