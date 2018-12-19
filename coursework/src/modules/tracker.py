#!/usr/bin/env python
import roslib
import rospy
import tf
import time
import math
import numpy as np
from goToPoint import GoToPose
from ar_track_alvar_msgs.msg import AlvarMarkers
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Bool

class Tracker():

	def __init__(self):
		self.tf_listener = tf.TransformListener()
		self.length = 0.4
		self.arfound = False
		self.navigate = GoToPose()
		self.arlist = []
		self.postercounter = 0
		self.posterx = 0
		self.postery = 0
		self.posterz = 0
		self.quatList =[]
		self.alvar_sub = rospy.Subscriber("ar_pose_marker", AlvarMarkers, self.findar)
		# self.image_sub = rospy.Subscriber('Cluedo_Result', String, self.callback)
		self.arDetectPub = rospy.Publisher('CluARFound', Bool, queue_size=10)

	#find ar_marker then
	def findar(self,markers):
		# while ar marker is not registered
		while not self.arfound:
			# self.tf_listener.waitForTransform("/map", "/ar_marker_0", rospy.Time(0), rospy.Duration(4.0))
			try :
				(trans,rot) = self.tf_listener.lookupTransform('/map', '/ar_marker_0', rospy.Time(0))
			except :
				return
			self.posterx,self.postery,self.posterz = trans[0],trans[1],trans[2]
			#check if marker already registered
			if len(self.arlist) > 0:
				for i in self.arlist:
					for j in i:
						samePoster = self.nearequal(self.posterx, i[0], self.postery, i[1], 0.2)
						if samePoster:
							continue
						else:
							#new poster found
							self.addPoster(trans, rot)
							self.postercounter += 1
			#if poster is in new position
			else:
				#new poster found
				self.addPoster(trans, rot)
				self.postercounter += 1
			rospy.sleep(1)
		return self.arfound

	def addPoster(self, trans, rot):
		self.arlist.append(trans)
		self.quatList.append(rot)

	def look(self, posterNumber):
		trans = self.arlist[posterNumber]
		rot = self.quatList[posterNumber]

		#predefined optimal position for robot relative to poster
		Tav = np.matrix([[0, 0, 1, 0],
						 [0, 1, 0, 0],
						 [-1, 0, 0, 0.5],
						 [0, 0, 0, 1]])

		#matrix containing ar_marker position in map coordinate framework
		Tma = self.tf_listener.fromTranslationRotation(trans, rot)

		#get matrix map to suitable position in front of the poster
		Tmv = Tma * Tav

		#get position and quaternion from matrix
		position = {'x': Tmv[0,3], 'y' : Tmv[1,3]}
		quat = tf.transformations.quaternion_from_matrix(Tmv)
		quaternion = {'r1' : quat[0], 'r2' : quat[1], 'r3' : quat[2], 'r4' : quat[3]}

		#send command to move to the poster
		return self.navigate.goto(position,quaternion)

	def position(self, posterId):
		#navigate to poster
		reachDest = self.look(posterId);
		if(reachDest):
			#save poster coordinates
			# print self.arlist
			self.arDetectPub.publish(self.arfound)
			self.arfound = False
			rospy.sleep(1)
			return True
		return False

	def nearequal(self,x1,x2,y1,y2,tolerance):
		if abs(x1-x2) < tolerance and abs(y1-y2) < tolerance:
			return True
		else:
			return False

	def postercount(self):
		return self.postercounter

	def getpostercoordinates(self,x):
		return self.arlist[x]
