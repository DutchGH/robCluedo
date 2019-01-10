#!/usr/bin/env python
from __future__ import division
import os

import cv2
import numpy as np
import rospy
import sys
import tensorflow as tf
import copy
import uuid

from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import Image
from std_msgs.msg import String, Bool
from cv_bridge import CvBridge, CvBridgeError

class robCluedo:
    def __init__(self, pub, rate):
        self.image_sub = rospy.Subscriber('CluARFound', Bool, self.callback)
        self.image_feed = rospy.Subscriber('/camera/rgb/image_raw/', Image, self.setRawImage)
        self.movement_pub = rospy.Publisher('mobile_base/commands/velocity', Twist, queue_size=10)
    	self.rate = rospy.Rate(10) #10hz
        self.desired_velocity = Twist()

        self.rawImage = None
        self.murderer = CleudoCharacter("Unkown","","")
        self.murderWeapon = CleudoCharacter("Unkown","","")

        print("Up and Running")

    def callback(self, data):
        print(data.data)

    def ImageResult(self, data):
        print(data.data)

    def setRawImage(self, data):
        self.rawImage = CvBridge().imgmsg_to_cv2(data, "bgr8")

    def getRawImage(self):
        return self.rawImage
    
    def centerImage(self):
        x = 640
        # Convert To Canny Edge Detection
        img_canny = cv2.Canny(self.getRawImage(), 100, 200)
        (contours, _) = cv2.findContours(img_canny, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Find the largest area for the contour
        cnt_areas = [cv2.contourArea(contours[n]) for n in range(len(contours))]
        maxCnt_index = cnt_areas.index(max(cnt_areas))

        M = cv2.moments(contours[maxCnt_index])
        max_cnt_x = int(M['m10']/M['m00'])
        max_cnt_y = int(M['m01']/M['m00'])

        # Centre image within the frame
        img_centered = False
        if x - max_cnt_x > 340:
            self.desired_velocity.angular.z = 0.2

        elif x - max_cnt_x < 280:
            self.desired_velocity.angular.z = -0.2

        else:
            # Stop rotation (centering completed)
            self.desired_velocity.angular.z = 0
            img_centered = True

        # Publish velocity and tell whoever called us if we've centered or not
        self.movement_pub.publish(self.desired_velocity)
        return img_centered
        

    def assignScannedImage(self, clu):
        if clu.category == "PERSON":
            if self.murderer is None:
                self.murderer = clu
            else:
                if self.murderer.getScore() < clu.getScore():
                    self.murderer = clu
                else:
                    print("We already have a murderer assigned.")
        elif clu.category == "WEAPON":
            if self.murderWeapon is None:
                self.murderWeapon = clu
            else:
                if self.murderWeapon.getScore() < clu.getScore():
                    self.murderWeapon = clu
                else:
                    print("We already have a murder weapon assigned.")
        else:
            print("This character does not have a type, we won't assign it")

    def writeResultsFile(self):
        resultsDir = os.path.dirname(os.path.abspath(__file__)) + "/../../results/"
        if not os.path.exists(resultsDir):
            os.makedirs(resultsDir)
        resFile = resultsDir + "CluedoResults.txt"
        with open(resFile, 'w') as fp:
            fp.write("RESULTS\n")
            fp.write("Murderer:\n")
            fp.write("Name: " + self.murderer.name + "\n")
            fp.write("Location: " + self.murderer.getLocation() + "\n")
            fp.write("Image Location: " + self.murderer.getImageLocation() + "\n")
            fp.write("\n")
            fp.write("Murder Weapon:\n")
            fp.write("Type: " + self.murderWeapon.name + "\n")
            fp.write("Location: " + self.murderWeapon.getLocation() + "\n")
            fp.write("Image Location: " + self.murderWeapon.getImageLocation() + "\n")
            fp.close()
        print("Finished! Results can be found at: " + str(os.path.normpath(resFile)))

class CleudoCharacter:
    def __init__(self, name, fn, cat):
        self.name = name
        self.fn = fn
        self.category = cat
        self.templateScore = 0
        self.location = ""
        self.imageLocation = ""

    def getScore(self):
        return self.templateScore

    def setScore(self, score):
        self.templateScore = score

    def getCategory(self):
        return self.category

    def setLocation(self, loc):
        self.location = loc

    def setImageLocation(self, loc):
        self.imageLocation = loc

    def getImageLocation(self):
        return self.imageLocation

    def getLocation(self):
        return self.location

class CluedoClassifier():

    def __init__(self):
        # Initialise a publisher to publish messages to the robot base
        # We covered which topic receives messages that move the robot in the 2nd Lab Session
        self.clu_list = createCharacterList()

    def analyseImg(self, img):
        cv_image = img
        clu_list = self.clu_list
        bestCharacter = CleudoCharacter(None, None, None)

        conv = np.float32(cv_image)
        img = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        cv2.normalize(img, img, 0, 255, cv2.NORM_MINMAX)
        img2 = img.copy()
        top_left = None
        best_w = 0
        best_h = 0
        for clu in clu_list:
            scale = 0.3
            # if clu.category == "WEAPON":
            #     scale = 0.4
            template = cv2.imread(str(clu.fn),0)
<<<<<<< Updated upstream
            print(str(clu.fn))
            template = cv2.resize(template, None, fx = scale, fy = scale, interpolation = cv2.INTER_CUBIC)
=======
            template = cv2.resize(template, None, fx = 0.3, fy = 0.3, interpolation = cv2.INTER_CUBIC)
>>>>>>> Stashed changes
            cv2.normalize(template, template, 0, 255, cv2.NORM_MINMAX)
            # template = cv2.Canny(template, 50,200)
            w, h = template.shape[::-1]

            meth = 'cv2.TM_CCOEFF_NORMED'
            # meth = 'cv2.TM_CCOEFF'


            img = img2.copy()
            # img = cv2.Canny(img, 100,200)
            method = eval(meth)

            # Apply template Matching
            res = cv2.matchTemplate(img,template,method)
            min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(res)
            clu.setScore(max_val)
            if clu.getScore() > bestCharacter.getScore(): # and clu.getScore() > 0.5
                bestCharacter = clu
                top_left = max_loc
                best_w = w
                best_h = h


        print (bestCharacter.name, bestCharacter.getScore())
        bottom_right = (top_left[0] + w, top_left[1] + h)
        cv2.rectangle(img, top_left, bottom_right, 255,0,0,0)
        # newFile = os.path.dirname(os.path.abspath(__file__)) + "/savedimg/" + str(uuid.uuid4()) + ".jpg"
        imageDir = os.path.dirname(os.path.abspath(__file__)) + "/../../results/" + "capturedImages/"
        newFile = imageDir + bestCharacter.name + ".jpg"
        if not os.path.exists(imageDir):
            os.makedirs(imageDir)
        cv2.imwrite(newFile, img)
        sanFile = str(os.path.normpath(newFile))
        bestCharacter.setImageLocation(sanFile)
        return bestCharacter

        # cv2.waitKey(3)


def createCharacterList():
    clu_list = []
    clu_list.append(CleudoCharacter("Mustard", os.path.dirname(os.path.abspath(__file__)) + '/templates/mustard.png', "PERSON"))
    clu_list.append(CleudoCharacter("Peacock", os.path.dirname(os.path.abspath(__file__)) +'/templates/peacock.png',"PERSON"))
    clu_list.append(CleudoCharacter("Scarlet", os.path.dirname(os.path.abspath(__file__)) +'/templates/scarlet.png',"PERSON"))
    clu_list.append(CleudoCharacter("Plum", os.path.dirname(os.path.abspath(__file__)) +'/templates/plum.png',"PERSON"))
    clu_list.append(CleudoCharacter("Wrench", os.path.dirname(os.path.abspath(__file__)) +'/templates/wrench.png',"WEAPON"))
    clu_list.append(CleudoCharacter("Rope", os.path.dirname(os.path.abspath(__file__)) +'/templates/rope.png',"WEAPON"))
    clu_list.append(CleudoCharacter("Revolver", os.path.dirname(os.path.abspath(__file__)) +'/templates/revolver.png',"WEAPON"))

    return clu_list
