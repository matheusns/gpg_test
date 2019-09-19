#!/usr/bin/env python

import angles as ros_angles
import tf
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose2D
import rospy
from nav_msgs.msg import Odometry

from math import pow, atan2, sqrt

class lineSensor:
    def __init__(self):
        rospy.init_node('turtle_pose_goal_sender')
        self.initMemberVariables()
        self.initROSChannels()
        self.randomGoalSender()

    def initMemberVariables(self):
        self.current_pose = Odometry()

    def initROSChannels(self):
        self.pose_listener = rospy.Subscriber("gpg/mobile_base_controller/odom", Odometry, self.callbackPose)
        self.goal_publisher = rospy.Publisher("gpg/goal", Pose2D, queue_size=10)

    def callbackPose(self, msg):
        self.current_pose = msg