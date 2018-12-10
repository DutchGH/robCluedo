#!/usr/bin/env python
import roslib
import rospy
import tf
import time
import math
import numpy as np
from go_to_specific_point_on_map import GoToPose 
from ar_track_alvar_msgs.msg import AlvarMarkers
from geometry_msgs.msg import Twist

class Tracker():
	
	def __init__(self):
		self.alvar_sub = rospy.Subscriber("ar_pose_marker", AlvarMarkers, self.findar)
		self.tf_listener = tf.TransformListener()
		self.length = 0.4
		self.navigate = GoToPose()
		self.arfound = False
		
	#find ar_marker then 
	def findar(self,markers):
		for m in markers.markers:
			p = m.pose.pose
			#posterx,postery,posterz = p.position.x, p.position.y, p.position.z
			self.arfound = True

			rospy.sleep(1)
			
		return



	def rotate(self):
		pub = rospy.Publisher('mobile_base/commands/velocity', Twist, queue_size=10)
		rate = rospy.Rate(10) #10hz
		desired_velocity = Twist()
		desired_velocity.linear.x = 0.0 # Forward
		desired_velocity.angular.z = 0.2 # Rotate
		while not self.arfound:	#change!!
			pub.publish(desired_velocity)
			
		
	def look(self):
		self.rotate()
		
		#get poster position
		self.tf_listener.waitForTransform("/map", "/ar_marker_0", rospy.Time(0), rospy.Duration(4.0))
		(trans,rot) = self.tf_listener.lookupTransform('/map', '/ar_marker_0', rospy.Time(0))
		
		#predefined optimal position for robot relative to poster
		Tav = np.matrix([[0, 0, 1, 0], 
						[0, 1, 0, 0],
						[-1, 0, 0, 0.4],
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
		self.navigate.goto(position,quaternion)
		
		
	def position(self):
		self.look()

def main():
	rospy.init_node('ar_tracker', anonymous=True, log_level=rospy.INFO)
	try:
		tracker = Tracker()
		tracker.position()
	except rospy.ROSInterruptException:
		pass

if __name__ == '__main__':
	main()
	
