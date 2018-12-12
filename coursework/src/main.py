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


from modules import Tracker

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

def main(args):
    rospy.init_node('cluedo_main', anonymous=True)
    pub, rate = publisher("CluedoMain", Bool)
    cI = robCluedo(pub, rate)
    while not rospy.is_shutdown():
        try:
            tracker = Tracker()
            while tracker.postercount() < 2:
				#change to navigate around the map
                tracker.rotate()
                #if arfound is true this means that new poster is found
                #and it needs to move to that location
                if tracker.arfound:
					#returns true when position reached for recognition
					tracker.position()
            
			
            
            rospy.spin()
        except KeyboardInterrupt:
            print("Shutting down")

if __name__ == "__main__":
    main(sys.argv)
