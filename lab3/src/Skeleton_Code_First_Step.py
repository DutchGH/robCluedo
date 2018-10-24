#!/usr/bin/env python

from __future__ import division
import cv2
import numpy as np
import rospy
import sys

from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class colourIdentifier():

	def __init__(self):
		# Remember to initialise a CvBridge() and set up a subscriber to the image topic you wish to use
		rospy.bridge = CvBridge()
		rospy.Subscriber('camera/rgb/image_raw', Image, self.callback)
		# We covered which topic to subscribe to should you wish to receive image data
	
	def callback(self, data):
		# Convert the received image into a opencv image
		# But remember that you should always wrap a call to this conversion method in an exception handler
		try:
			cv_image = rospy.bridge.imgmsg_to_cv2(data, "bgr8")
		except CvBridgeError as e:
			print(e)		
		#Show the resultant images you have created. You can show all of them or just the end result if you wish to.
		cv2.namedWindow('Camera_Feed')
		hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
		sensitivity = 10
		hsv_green_lower = np.array([60 - sensitivity, 100, 100])
		hsv_green_upper = np.array([60 + sensitivity, 255, 255])
		mask = cv2.inRange(hsv, hsv_green_lower, hsv_green_upper)
		output = cv2.bitwise_and(cv_image, cv_image, mask=mask)
		
		cv2.imshow('Camera_Feed', output)
		cv2.waitKey(0)
		
# Create a node of your class in the main and ensure it stays up and running
# handling exceptions and such
def main(args):
	# Instantiate your class
	# And rospy.init the entire node
	cI = colourIdentifier()
	rospy.init_node('colourIdentifier', anonymous=True)
	# Ensure that the node continues running with rospy.spin()
	# You may need to wrap rospy.spin() in an exception handler in case of KeyboardInterrupts
	try:
		rospy.spin()
	except KeyboardInterrupt:
		print("Shutting down")
	# Remember to destroy all image windows before closing node
	cv2.destroyAllWindows()
	
# Check if the node is executing in the main path
if __name__ == '__main__':
	main(sys.argv)


