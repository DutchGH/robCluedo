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
from cv_bridge import CvBridge, CvBridgeError

from std_msgs.msg import String, Bool
from modules import robotStatus
from modules import Tracker
from modules import CluedoClassifier
from modules import FollowWall

class robCluedo:
    def __init__(self, pub, rate):
        self.image_sub = rospy.Subscriber('CluARFound', Bool, self.callback)
        self.image_feed = rospy.Subscriber('/camera/rgb/image_raw/', Image, self.setRawImage)

        self.rawImage = None
        self.murderer = None
        self.murderWeapon = None

        print("Up and Running")

    def callback(self, data):
        print(data.data)

    def ImageResult(self, data):
        print(data.data)

    def setRawImage(self, data):
        self.rawImage = CvBridge().imgmsg_to_cv2(data, "bgr8")

    def getRawImage(self):
        return self.rawImage

def publisher(name, type):
	pub = rospy.Publisher(name, type, queue_size=10)
	rate = rospy.Rate(10) #10hz

	return pub, rate

def main():
    rospy.init_node('cluedo_main', anonymous=True)
    pub, rate = publisher("CluedoMain", Bool)
    cI = robCluedo(pub, rate)
    wallfollower = FollowWall()
    running = True
	#start
    try:
        robotRunning = robotStatus.RobotStatus()
        #got to middle of the room
        robotRunning.goToMiddle()
        while running:
            # if both posters are found
            if robotRunning.tracker.postercounter == 2:
                print(' Found both poster from the middle of the room ')
                for i in range(0,2):
                    scanposter(robotRunning,cI,i)
                # end program
                running = False
                break

            #only single poster was found
            if robotRunning.tracker.postercounter == 1:
                robotRunning.stopMovement()
                print('Scan first poster')
                scanposter(robotRunning,cI,0)
                robotRunning.goToEntrance()
                # wallfollower.start()
                wallfollower.start()
                print('starting search for second poster')
                while robotRunning.tracker.postercounter < 3:
                    if robotRunning.tracker.postercounter == 2:
                        print('found second')
                        wallfollower.stop()
                        robotRunning.stopMovement()
                        scanposter(robotRunning,cI,1)
                        running = False
                        break

            # No posters identified after initial spin in the middle of the room
            else:
                robotRunning.goToEntrance()
                # start following wall
                # wallfollower.startCounter()
                wallfollower.start()

                doneOnce = False
                while robotRunning.tracker.postercounter < 3:
                    if robotRunning.tracker.postercounter == 1 and doneOnce == False:
                        doneOnce = True
                        print('found ar marker - count = 1')
                        wallfollower.stop()
                        robotRunning.stopMovement()
                        scanposter(robotRunning,cI,0)
                        print('starting search for second poster')
                        # wallfollower.startCounter()
                        wallfollower.start()

                    elif robotRunning.tracker.postercounter == 2:
                        print('poster counter = 2')
                        wallfollower.stop()
                        robotRunning.stopMovement()
                        scanposter(robotRunning,cI,1)
                        running = False
                        break

    except KeyboardInterrupt:
        print("Shutting down")

def scanposter(robotRunning,cI,i):
    goToDest = robotRunning.tracker.position(i)
    if goToDest:
        print('Scanning poster')
        #This will print out a "CluedoCharacter Object, You can get stuff like name, type etc"
        data = robotRunning.cluedoClassifier.analyseImg(cI.getRawImage())
        print(data)
        print(data.name)
        print('finished image analysis')
        # save image

if __name__ == "__main__":
    main()
