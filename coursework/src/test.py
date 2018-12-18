#!/usr/bin/env python

import roslib
import rospy
import numpy as np
import time
import random
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32

class FollowWall:
    def __init__(self):
        rospy.init_node('WallFollowerNode')

        # subscribe to laserscan topic
        rospy.Subscriber('/scan', LaserScan, self.laserscan_callback)
        self.velocityPublish = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size = 10)

        self.rate = rospy.Rate(10)
        self.velocity = Twist()
        self.laserscan = None

        self.angle = 1.57284
        self.centre = Float32()
        self.left = Float32()
        self.right = Float32()

    def switch_movement(self, direction):
        # direction
        self.switch = {
            1: "Ahead",
            2: "Left",
            3: "Right"
        }
        return self.switch.get(direction, "nothing")

    def laserscan_callback(self, laser_data):
        # set laserscan ranges for middle, left and right to robot
        self.centre.data= laser_data.ranges[320]
        rospy.loginfo("centre %f", laser_data.ranges[320])
        self.left.data= laser_data.ranges[639]
        rospy.loginfo("left %f", laser_data.ranges[639])
        self.right.data= laser_data.ranges[0]
        rospy.loginfo("right %f", laser_data.ranges[0])
        self.laserscan = laser_data

    def start(self):
        while not rospy.is_shutdown():
            if (self.laserscan):
                # minLaserValue = min(min(self.laserscan.ranges[170:190], self.laserscan.ranges[150:170]))
                # self.findClosestWall()
                self.followWall('Left')

            self.rate.sleep()

    def findClosestWall(self):
        # find closest wall and go to it
        # cases in which one of the values cannot be defined
        if (np.isnan(self.centre.data)):
            if (self.left.data < self.right.data):
                # closest wall is on the left
                print(self.switch_movement(2))
                self.followWall(self.switch_movement(2))
            else:
                # closest wall is on the right
                print(self.switch_movement(3))
                self.followWall(self.switch_movement(3))
        elif (np.isnan(self.left.data)):
            if (self.centre.data < self.right.data):
                # closest wall is ahead
                print(self.switch_movement(1))
                self.followWall(self.switch_movement(1))
            else:
                # closest wall is on the right
                print(self.switch_movement(3))
                self.followWall(self.switch_movement(3))
        elif (np.isnan(self.right.data)):
            if (self.centre.data < self.left.data):
                # closest wall is ahead
                print(self.switch_movement(1))
                self.followWall(self.switch_movement(1))
            else:
                # closest wall is on the left
                print(self.switch_movement(2))
                self.followWall(self.switch_movement(2))
        else:
            # all values can be defined
            # find smallest value
            if (self.centre.data < self.left.data and self.centre.data < self.right.data):
                # closest wall is ahead
                print(self.switch_movement(1))
                self.followWall(self.switch_movement(1))
            elif (self.left.data < self.centre.data and self.left.data < self.right.data):
                # closest wall is on the left
                print(self.switch_movement(2))
                self.followWall(self.switch_movement(2))
            elif (self.right.data < self.centre.data and self.right.data < self.left.data):
                # closest wall is on the right
                print(self.switch_movement(3))
                self.followWall(self.switch_movement(3))

    def followWall(self, direction):
        # follow closest wall
        if self.laserscan:
            if (direction == 'Left'):
                self.movementWallLeft()
            elif (direction == 'Ahead'):
                self.movementWallAhead()
            else:
                self.movementWallRight()

    # movement when the closest wall is ahead
    def movementWallAhead(self):
        if (self.centre.data > 1.7):
            # move forward
            print('centre > 0.8')
            self.moveForward()
            if (np.isnan(self.left.data)):
                if (np.isnan(self.right.data)):
                    # both right and left cannot be defined
                    print('move forward')
                    self.moveForward()
                else :
                    # left value cannot be defined
                    print ('left nan')
                    self.rotateRight()
            elif (np.isnan(self.right.data)):
                # right value cannot be defined
                print ('right nan')
                self.rotateLeft()
        else :
            # either too close to wall ahead or too far for scanner
            if (np.isnan(self.centre.data)):
                # too far to scan
                print(' centre nan ')
                self.moveForward()
            if (np.isnan(self.left.data)):
                if (np.isnan(self.right.data)):
                    # both right and left cannot be defined
                    print('move forward')
                    self.moveForward()
                else :
                    # left value cannot be defined
                    print ('left nan')
                    self.rotateLeft()
            elif (np.isnan(self.right.data)):
                print ('right nan')
                self.rotateRight()
            # elif (self.right.data < 0.9):
            #     print('left < right')
            #     self.rotateLeft()
            elif (self.left.data < self.right.data):
                print('left < right')
                if (self.left.data < 1.6):
                    print('left < 1.5')
                    self.rotateRight()
                else:
                    self.rotateLeft()
            else:
                print (' turn right ')
                self.rotateRight()
                rospy.sleep(0.6)

    # movement when the closest wall is to the left
    def movementWallLeft(self):
        if (self.centre.data > 1.8 or np.isnan(self.centre.data)):
            # move forward
            print('centre > 1')
            self.moveForward()
            if (self.left.data > 1.8 or np.isnan(self.left.data)):
                # too far from the wall
                print('left > 1.8')
                self.rotateLeft()
            elif (self.left.data < 0.6):
                # close to left object, turn right
                print('left < 0.8')
                self.rotateRight()
            elif (self.right.data < 0.3):
                # close to right object, turn left
                print('right < 0.8')
                self.rotateLeft()
        else :
            print(' else ')
            if (self.left.data > 0.7):
                print('left > 0.8')
                self.rotateLeft()
            # elif (self.right.data > 0.8):
            #     print('right > 0.8')
            #     self.rotateLeft()
            elif (self.left.data > self.right.data):
                print('left > right')
                self.rotateRight()
            else :
                print ('right > left')
                self.rotateLeft()
        #     # move forward
        #     print('centre > 1')
        #     self.moveForward()
        #     if (self.right.data < 0.3):
        #         # close to right object, turn left
        #         print('right < 0.3')
        #         self.rotateLeft()
        #     elif (self.left.data > 1.8):
        #         # too far from the wall
        #         print('left > 0.6')
        #         self.rotateLeft()
        #     elif (self.left.data < 0.6):
        #         # close to left object, turn right
        #         print('left < 0.4')
        #         self.rotateRight()
        #     else :
        #         print (' move forward ')
        #         self.rotateRight()
        #         self.stop()
        #         self.moveForward()
        # else :
        #     print(' else ')
        #     if (np.isnan(self.centre.data)):
        #         # if (np.isnan(self.right.data)):
        #         #     self.rotateRight()
        #         # else :
        #         # move Forward
        #         self.moveBackwards()
        #         self.rotateRight()
        #     elif (self.right.data < 1 and self.left.data < 1):
        #         # probably stuck in a corner
        #         print('stuck')
        #         self.moveBackwards()
        #     elif (self.left.data < 1.3 or np.isnan(self.left.data)):
        #         print('left < 0.4')
        #         self.rotateRight()
        #     elif (self.right.data < 1.2  or np.isnan(self.right.data)):
        #         print('right < 0.4')
        #         self.rotateLeft()
        #     else :
        #         print (' move back ')
        #         self.stuck_getOut()

    # movement when the closest wall is to the right
    def movementWallRight(self):
        if (self.centre.data > 1.8):
            # move forward
            print('centre > 1')
            self.moveForward()
            if (self.left.data < 0.3):
                # too close to left object, turn right
                print('left < 0.3')
                self.rotateRight()
            elif (self.right.data > 1.8):
                # too far from the wall, get closer
                print('right > 0.8')
                self.rotateRight()
            elif (self.right.data < 0.6):
                # too close to the wall, turn left
                print('right < 0.4')
                self.rotateLeft()
            else :
                print(' move forward ')
                self.rotateLeft()
                self.stop()
                self.rotateLeft()
                self.moveForward()
        else :
            print(' else ')
            if (np.isnan(self.centre.data)):
                # if (np.isnan(self.left.data)):
                #     # turn left
                #     self.rotateLeft()
                # else :
                # move back
                self.moveBackwards()
                self.rotateLeft()
            elif (self.right.data < 1.5 and self.left.data < 1.5):
                # probably stuck in a corner
                print('stuck')
                self.stuck_getOut()
            elif (self.right.data < 1.3 or np.isnan(self.right.data)):
                # too close to the wall, turn left
                print('right < 0.4')
                self.rotateLeft()
            elif (self.left.data < 1.2 or np.isnan(self.left.data)):
                # too close to the left, turn right
                print('left < 0.4')
                self.rotateRight()
            else :
                print (' move back ')
                self.stuck_getOut()

    # motion
    def stop(self):
        self.velocity.linear.x = 0.0
        self.velocity.angular.z = 0
        self.velocityPublish.publish(self.velocity)

    def moveForward(self):
        self.velocity.linear.x = 0.2
        self.velocity.angular.z = 0
        self.velocityPublish.publish(self.velocity)

    def moveBackwards(self):
        self.velocity.linear.x = -0.3
        self.velocity.angular.z = 0
        self.velocityPublish.publish(self.velocity)

    def rotateRight(self):
        self.velocity.linear.x = 0
        self.velocity.angular.z = -self.angle
        self.velocityPublish.publish(self.velocity)

    def rotateLeft(self):
        self.velocity.linear.x = 0
        self.velocity.angular.z = self.angle
        self.velocityPublish.publish(self.velocity)

    def stuck_getOut(self):
        # too close to both right side and left side, probably stuck
        # move back and rotate
        self.moveBackwards()
        rospy.sleep(0.6)
        # self.stop()
        if (self.left.data < self.right.data or np.isnan(self.left.data)):
            self.rotateRight()
            rospy.sleep(0.2)
            self.rotateRight()
        else :
            self.rotateLeft()
            rospy.sleep(0.2)
            self.rotateLeft()

if __name__ == '__main__':
    try:
        wallFollower = FollowWall()
        wallFollower.start()
    except rospy.ROSInterruptException:
		pass
