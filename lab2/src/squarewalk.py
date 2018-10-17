#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from kobuki_msgs.msg import BumperEvent 
from math import radians

def processBump(data):
	if (data.state == BumperEvent.PRESSED):
		global bump
		bump = True

def publisher():
	pub = rospy.Publisher('mobile_base/commands/velocity', Twist)
	rospy.init_node('Walker', anonymous=True)
	rate = rospy.Rate(10) #10hz
	desired_velocity = Twist()
	desired_velocity.linear.x = 0.2 # Forward with 0.2 m/sec.
	corner_turn = Twist()
	corner_turn.linear.x = 0
	corner_turn.angular.z = radians(45) 
	global bump
	bump = False
	rospy.Subscriber("mobile_base/events/bumper", BumperEvent, processBump)
	
	for j in range (4):
		for i in range (30):
			if bump==True:
				break
			pub.publish(desired_velocity)
			rate.sleep()	
		for i in range (20):
			if bump==True:
				break
			pub.publish(corner_turn)
			rate.sleep()
		if bump==True:
			break


if __name__ == "__main__":
	try:
		publisher()
	except rospy.ROSInterruptException:
		pass
