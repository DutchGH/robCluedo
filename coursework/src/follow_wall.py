#!/usr/bin/env python
import roslib
import rospy
import numpy as np
import math
from math import radians
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32

class FollowWall:
    def __init__(self):
        # subscribe to laserscan topic
        self.velocityPublish = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size = 10)

        self.velocity = Twist()

        # minimum distance from wall
        self.minDist_wall = 0.25
        # distance at middle of laserscan - in front of robot
        self.distAhead = 0
        self.speed = 0.2
        self.maxAngularVeloc = 1

        # angle against minimum distance point on map
        self.angle_minDist = 0
        self.error_value = 0
        self.d = 0.4
        self.diff_PrevError = 0

    def start(self):
        self.laserscanSubs = rospy.Subscriber('/scan', LaserScan, self.laserscan_callback)

    def stop(self):
        self.laserscanSubs.unregister()

    def laserscan_callback(self, scan_data):
        # append non nan values to a new list
        laserValues = []
        for i in xrange(0, len(scan_data.ranges)):
            if not np.isnan(scan_data.ranges[i]):
                laserValues.append(scan_data.ranges[i])

        size = len(laserValues)
        print(size)
        if (size == 0):
            print('size = 0')
            self.stop()
            self.stopMovement()
            self.reverseAndRotate()
            self.start()
        else :
            minIndex = int(size/2)
            self.distAhead = laserValues[minIndex]

            # find minimum index value
            for i in range(minIndex, size):
                if (laserValues[i] < laserValues[minIndex] and laserValues[i] > 0.01):
                    minIndex = i

            self.angle_minDist = (minIndex - size/2) * scan_data.angle_increment
            # scan value at minimum index
            minIndex_value = laserValues[minIndex]
            self.diff_PrevError = (minIndex_value - self.minDist_wall) - self.error_value
            self.error_value = minIndex_value - self.minDist_wall

            self.movement()

    def movement(self):
        velocity = (self.error_value + self.diff_PrevError) + (self.angle_minDist - math.pi * 0.5)
        if (velocity > 1):
            velocity = 1
        self.velocity.angular.z = velocity
        if (self.distAhead < self.minDist_wall):
            # too close to wall ahead, stop
            self.velocity.linear.x = 0
        elif (self.distAhead < self.minDist_wall * 2):
            # geeting close to wall, slow down
            self.velocity.linear.x = 0.5 * self.speed
        elif (abs(self.angle_minDist) > 1.75):
            self.velocity.linear.x = 0.4 * self.speed
        else:
            self.velocity.linear.x = self.speed

        self.velocityPublish.publish(self.velocity)

    def stopMovement(self):
        self.velocity.linear.x = 0
        self.velocity.angular.z = 0
        self.velocityPublish.publish(self.velocity)

    def reverseAndRotate(self):
        self.velocity.linear.x = -0.2
        self.velocityPublish.publish(self.velocity)
        rospy.sleep(0.5)
        self.stopMovement()
        self.rotate()
        rospy.sleep(1)

    def rotate(self):
        self.velocity.linear.x = 0
        self.velocity.angular.z = -radians(140)
        self.velocityPublish.publish(self.velocity)

if __name__ == '__main__':
    try:
        rospy.init_node('wallFollower', anonymous=True)
        wallFollower = FollowWall()
        wallFollower.start()
        # don't exit until stopped
        rospy.spin()
    except rospy.ROSInterruptException:
		pass
