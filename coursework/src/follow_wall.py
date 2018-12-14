#!/usr/bin/env python

import roslib
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class FollowWall:
    def __init__(self):
        rospy.init_node('WallFollowerNode')
        self.rate = rospy.Rate(10)

        rospy.Subscriber('/laserscan', LaserScan, self.laserscan_callback)
        self.velocityPublish = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size = 5)

        self.velocity = Twist()
        self.laserscan = None

    def laserscan_callback(self, laser_msg):
        self.laserscan = laser_msg

    def start(self):
        rospy.loginfo('Start Following Wall')

if __name__ == '__main__':
    print('starting')
    wallFollower = FollowWall()
    wallFollower.start();
