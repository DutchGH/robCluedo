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
from modules.cluedoClassifier import robCluedo
from modules import FollowWall


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
                # both posters scanned, end program, write the text file
                cI.writeResultsFile()
                running = False
                break

            #only single poster was found
            if robotRunning.tracker.postercounter == 1:
                robotRunning.stopMovement()
                print('Scan first poster')
                scanposter(robotRunning,cI,0)
                #the robot goes to the entrance of the world
                robotRunning.goToEntrance()
                wallfollower.start(robotRunning.entranceXcoordinate, robotRunning.entranceYcoordinate)
                print('starting search for second poster')
                while robotRunning.tracker.postercounter < 3:
                    if robotRunning.tracker.postercounter == 2:
                        # stop wall following and scan poster
                        print('found second')
                        wallfollower.stop()
                        robotRunning.stopMovement()
                        scanposter(robotRunning,cI,1)
                        # both posters scanned, end program, write the text file
                        cI.writeResultsFile()
                        running = False
                        break
            # No posters identified after initial spin in the middle of the room
            else:
                # the robot goes to the entrance of the world
                robotRunning.goToEntrance()
                # starts following the wall
                wallfollower.start(robotRunning.entranceXcoordinate, robotRunning.entranceYcoordinate)

                # doneOnce boolean is used to indicate the robot has found
                # and analysed the first image it becomes True
                # this is to avoid the algorithm analysing the same poster again
                doneOnce = False
                while robotRunning.tracker.postercounter < 3:
                    if robotRunning.tracker.postercounter == 1 and doneOnce == False:
                        doneOnce = True
                        print('found ar marker - count = 1')
                        # stop wall following and scan poster
                        wallfollower.stop()
                        robotRunning.stopMovement()
                        scanposter(robotRunning,cI,0)
                        print('starting search for second poster')
                        # resume wall following
                        wallfollower.start(robotRunning.entranceXcoordinate, robotRunning.entranceYcoordinate)
                    elif robotRunning.tracker.postercounter == 2:
                        print('poster counter = 2')
                        # stop wall following and scan poster
                        wallfollower.stop()
                        robotRunning.stopMovement()
                        scanposter(robotRunning,cI,1)
                        # both posters scanned, end program, write the text file
                        cI.writeResultsFile()
                        running = False
                        break

    except KeyboardInterrupt:
        print("Shutting down")

def scanposter(robotRunning,cI,i):
    goToDest = robotRunning.tracker.position(i)
    if goToDest:
        print('Scanning poster')
        while not cI.centerImage():
            print(".")
        #This will print out a "CluedoCharacter Object, You can get stuff like name, type etc"
        data = robotRunning.cluedoClassifier.analyseImg(cI.getRawImage())
        data.setLocation(str(robotRunning.tracker.arlist[i]))
        cI.assignScannedImage(data)
        print('finished image analysis')
        # save image




if __name__ == "__main__":
    main()
