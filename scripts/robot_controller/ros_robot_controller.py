#!/usr/bin/env python
# BSD 3-Clause License
#
# Copyright (c) 2019, Matheus Nascimento
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice, this
#    list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
#
# 3. Neither the name of the copyright holder nor the names of its
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

# Launch file created to attend the requirements established on the Ex4 by the discipline of Intelligent Control \
#      of Robotics Systems
# Professor: Wouter Caarls
# Students: Matheus do Nascimento Santos 1920858  (@matheusns)
#           Luciana Reys 1920856 (@lsnreys)

import angles as ros_angles
import tf
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose2D
import rospy
from nav_msgs.msg import Odometry

from math import pow, atan2, sqrt


class RobotController:
    def __init__(self):
        rospy.init_node('robot_pose_controller')
        self.initMemberVariables()
        self.initROSChannels()
        rospy.spin()

    def initROSChannels(self):
        # The following lines are respective to exercise 2 item 'c'
        self.pose_listener = rospy.Subscriber("gpg/mobile_base_controller/odom", Odometry, self.callbackPose)
        self.goal_listener = rospy.Subscriber("gpg/goal", Pose2D, self.callbackGoal)
        self.velocity_publisher = rospy.Publisher('gpg/mobile_base_controller/cmd_vel', Twist, queue_size=10)

    def initMemberVariables(self):
        self.current_pose = Odometry()

    def callbackPose(self, msg):
        self.current_pose = msg

    def callbackGoal(self, msg):
        current_goal = msg
        self.goToGoal(current_goal)

    # The following lines are respective to exercise 2 item 'd'
    def goToGoal(self, goal_position):
        rospy.loginfo("Moving robot to the pose: (" + str(goal_position.x) + " , " + str(
            goal_position.y) + ")" + " and theta = " + str(goal_position.theta))

        vel_msg = Twist()

        vel_msg.linear.y = 0
        vel_msg.linear.z = 0

        vel_msg.angular.x = 0
        vel_msg.angular.y = 0

        distance_tolerance = 0.2

        rate = rospy.Rate(20)

        while self.euclideanDistance(goal_position) >= distance_tolerance:

            vel_msg.linear.x = self.linearVelocity(goal_position)
            vel_msg.angular.z = self.angularVelocity(goal_position)

            self.velocity_publisher.publish(vel_msg)
            rate.sleep()

        vel_msg.linear.x = 0
        vel_msg.angular.z = 0

        self.velocity_publisher.publish(vel_msg)

    def euclideanDistance(self, goal_pose):
        return sqrt(pow((goal_pose.x - self.current_pose.pose.pose.position.x), 2) + pow((goal_pose.y - self.current_pose.pose.pose.position.y), 2))

    def linearVelocity(self, goal_pose, gain=1):
        return gain * self.euclideanDistance(goal_pose)

    def angularVelocity(self, goal_pose, gain=4):
        return gain * ros_angles.shortest_angular_distance(self.yawFromQuaternion(self.current_pose.pose.pose.orientation), self.angleToTheGoal(goal_pose))

    def yawFromQuaternion(self, quaternionMsg):

        quaternionList = self.quaternionMsg2List(quaternionMsg)
        euler = tf.transformations.euler_from_quaternion(quaternionList)
        return euler[2]

    def quaternionMsg2List(self, quaternionMsg):
        quaternion = (quaternionMsg.x, quaternionMsg.y, quaternionMsg.z, quaternionMsg.w)
        return quaternion

    def angleToTheGoal(self, goal_pose):
        return atan2(goal_pose.y - self.current_pose.pose.pose.position.y, goal_pose.x - self.current_pose.pose.pose.position.x)
