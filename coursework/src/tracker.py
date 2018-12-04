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

		#get poster rotation
		euler = tf.transformations.euler_from_quaternion(rot)

		#get optimal robot position
		x = trans[0] - (np.cos(euler[2]) * self.length)
		y = trans[1] - (np.sin(euler[2]) * self.length)

        #get optimal robot rotation
		theta = math.atan2(trans[1]-y,trans[0]-x)

		#get position and quaternion to send to the robot
		position = {'x': x, 'y' : y}
		quaternion = {'r1' : 0.00, 'r2' : 0.00, 'r3' : np.sin(theta/2.0), 'r4' : np.cos(theta/2.0)}
		
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
	
