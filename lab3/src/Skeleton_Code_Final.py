#!/usr/bin/env python
# This final piece fo skeleton code will be centred around gettign the students to follow a colour and stop upon sight of another one.

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
		# Initialise a publisher to publish messages to the robot base
		# We covered which topic receives messages that move the robot in the 2nd Lab Session
		self.pub = rospy.Publisher('mobile_base/commands/velocity', Twist, queue_size=10)
		
		# Initialise any flags that signal a colour has been detected in view
		self.red_visible = 0
		self.green_visible = 0
		self.blue_visible = 0

		# Initialise the value you wish to use for sensitivity in the colour detection (10 should be enough)
		rospy.sensitivity = 20

		# Initialise some standard movement messages such as a simple move forward and a message with all zeroes (stop)
		rospy.moveforward = Twist()
		rospy.moveforward.linear.x = 0.2
		
		rospy.moveback = Twist()
		rospy.moveback.linear.x = - 0.2
		
		rospy.turnl = Twist()
		rospy.turnl.linear.x = 0
		rospy.turnl.angular.z = 1.508
		
		rospy.turnr = Twist()
		rospy.turnr.linear.x = 0
		rospy.turnr.angular.z = -1.508
		
		rospy.stopmoving = Twist()
		rospy.stopmoving.linear.x = 0
		rospy.stopmoving.angular.z = 0
		
		# Remember to initialise a CvBridge() and set up a subscriber to the image topic you wish to use
		rospy.bridge = CvBridge()
		# We covered which topic to subscribe to should you wish to receive image data
		rospy.Subscriber('camera/rgb/image_raw', Image, self.callback)
				
	def callback(self, data):
		# Convert the received image into a opencv image
		# But remember that you should always wrap a call to this conversion method in an exception handler
		try:
			cv_image = rospy.bridge.imgmsg_to_cv2(data, "bgr8")
		except CvBridgeError as e:
			print(e)
		
		# Set the upper and lower bounds for the two colours you wish to identify
		hsv_colour1_lower = np.array([60 - rospy.sensitivity, 100, 100])#green
		hsv_colour1_upper = np.array([60 + rospy.sensitivity, 255, 255])
		hsv_colour2_lower = np.array([120 - rospy.sensitivity, 100, 100])#blue
		hsv_colour2_upper = np.array([120 + rospy.sensitivity, 255, 255])
		hsv_colour3_lower = np.array([0 - rospy.sensitivity, 100, 100])#red
		hsv_colour3_upper = np.array([0 + rospy.sensitivity, 255, 255])
		
		# Convert the rgb image into a hsv image
		hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
		
		# Filter out everything but particular colours using the cv2.inRange() method
		# Be sure to do this for the second or third colour as well
		mask1 = cv2.inRange(hsv_image, hsv_colour1_lower, hsv_colour1_upper)
		mask2 = cv2.inRange(hsv_image, hsv_colour2_lower, hsv_colour2_upper)
		mask3 = cv2.inRange(hsv_image, hsv_colour3_lower, hsv_colour3_upper)

		# To combine the masks you should use the cv2.bitwise_or() method
		# You can only bitwise_or two image at once, so multiple calls are necessary for more than two colours
		mask1_2 = cv2.bitwise_or(mask1, mask2)
		masks = cv2.bitwise_or(mask1_2, mask3)
		
		# Apply the mask to the original image using the cv2.bitwise_and() method
		# As mentioned on the worksheet the best way to do this is to bitwise and an image with itself and pass the mask to the mask parameter
		# As opposed to performing a bitwise_and on the mask and the image. 
		output = cv2.bitwise_and(cv_image, cv_image, mask=masks)

		
		# Find the contours that appear within the certain colours mask using the cv2.findContours() method
		# For <mode> use cv2.RETR_LIST for <method> use cv2.CHAIN_APPROX_SIMPLE
		contours1 = cv2.findContours(mask1 ,cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)[0]
		contours2 = cv2.findContours(mask2 ,cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)[0]
		contours3 = cv2.findContours(mask3 ,cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)[0]

		# Loop over the contours
		# There are a few different methods for identifying which contour is the biggest
		# Loop throguht the list and keep track of which contour is biggest or
		# Use the max() method to find the largest contour
		# M = cv2.moments(c)
		# cx, cy = int(M['m10']/M['m00']), int(M['m01']/M['m00'])
		
		colour_max_area = 0
		
		#GREEN
		if len(contours1) != 0:
			c = max(contours1, key = cv2.contourArea)
			M = cv2.moments(c)
			cx, cy = int(M['m10']/M['m00']), int(M['m01']/M['m00'])
			colour_max_area = cv2.contourArea(c)
	
		#Check if the area of the shape you want is big enough to be considered
		# If it is then change the flag for that colour to be True(1)
		if colour_max_area > 1000:
			# draw a circle on the contour you're identifying as a blue object as well
			# cv2.circle(<image>, (<center x>,<center y>), <radius>, <colour (rgb tuple)>, <thickness (defaults to 1)>)
			# Then alter the values of any flags
			(x,y),radius = cv2.minEnclosingCircle(c)
			radius = int(radius)
			circle = cv2.circle(output, (cx, cy), radius, (255,0,0) ,1)
			output = cv2.bitwise_and(output,output , mask=circle)
			self.green_visible = 1
		else:
			self.green_visible = 0
			
		colour_max_area = 0
		#BLUE	
		if len(contours2) != 0:
			c = max(contours2, key = cv2.contourArea)
			M = cv2.moments(c)
			cx, cy = int(M['m10']/M['m00']), int(M['m01']/M['m00'])
			colour_max_area = cv2.contourArea(c)
	
		#Check if the area of the shape you want is big enough to be considered
		# If it is then change the flag for that colour to be True(1)
		if colour_max_area > 1000:
			# draw a circle on the contour you're identifying as a blue object as well
			# cv2.circle(<image>, (<center x>,<center y>), <radius>, <colour (rgb tuple)>, <thickness (defaults to 1)>)
			# Then alter the values of any flags
			(x,y),radius = cv2.minEnclosingCircle(c)
			radius = int(radius)
			circle = cv2.circle(output, (cx, cy), radius, (255,0,0) ,1)
			output = cv2.bitwise_and(output,output , mask=circle)
			self.blue_visible = 1
		else:
			self.blue_visible = 0
			
		colour_max_area = 0	
		#RED	
		if len(contours3) != 0:
			c = max(contours3, key = cv2.contourArea)
			M = cv2.moments(c)
			cx, cy = int(M['m10']/M['m00']), int(M['m01']/M['m00'])
			colour_max_area = cv2.contourArea(c)
	
		#Check if the area of the shape you want is big enough to be considered
		# If it is then change the flag for that colour to be True(1)
		if colour_max_area > 1000:
			# draw a circle on the contour you're identifying as a blue object as well
			# cv2.circle(<image>, (<center x>,<center y>), <radius>, <colour (rgb tuple)>, <thickness (defaults to 1)>)
			# Then alter the values of any flags
			(x,y),radius = cv2.minEnclosingCircle(c)
			radius = int(radius)
			circle = cv2.circle(output, (cx, cy), radius, (255,0,0) ,1)
			output = cv2.bitwise_and(output,output , mask=circle)
			self.red_visible = 1
		else:
			self.red_visible = 0

		#Check if a flag has been set for the stop message
		if self.red_visible == 1:
			if (colour_max_area) > 3000:
				# Too close to object, need to move backwards
				# linear = positive
				# angular = radius of minimum enclosing circle
				self.pub.publish(rospy.moveback)
			elif (colour_max_area) < 3000:
				# Too far away from object, need to move forwards
				# linear = positive
				# angular = radius of minimum enclosing circle
				self.pub.publish(rospy.moveforward)
		else:
			self.pub.publish(rospy.stopmoving)
			

		# Be sure to do this for the other colour as well
		#Show the resultant images you have created. You can show all of them or just the end result if you wish to.
		cv2.namedWindow('Camera_Feed')
		cv2.imshow('Camera_Feed', output)
		cv2.waitKey(3)

# Create a node of your class in the main and ensure it stays up and running
# handling exceptions and such
def main(args):
	# Instantiate your class
	# And rospy.init the entire node
	cI = colourIdentifier()
	rospy.init_node('colourIdentifier', anonymous=True)
	# Ensure that the node continues running with rospy.spin()
	# You may need to wrap it in an exception handler in case of KeyboardInterrupts
	try:
		rospy.spin()
	except KeyboardInterrupt:
		print("Shutting down")
	# Remember to destroy all image windows before closing node
	cv2.destroyAllWindows()

# Check if the node is executing in the main path
if __name__ == '__main__':
	main(sys.argv)


