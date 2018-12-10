#!/usr/bin/env/python

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

from modules import Tracker

class robCluedo:
    def __init__(self):
        print("Up and Running")

def main(args):
    print "We are Operational!"

if __name__ == "__main__":
    main(sys.argv)
