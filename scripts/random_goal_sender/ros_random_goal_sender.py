#!/usr/bin/env python
# BSD 3-Clause License
#
# Copyright (c) 2019, Matheus Nascimento, Luciana Reys
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

from math import pow, sqrt, pi
import random

from geometry_msgs.msg import Pose2D
import rospy
from nav_msgs.msg import Odometry


class GoalSender:
    def __init__(self):
        rospy.init_node('random_goal_sender')
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

    def randomGoalSender(self):
        r = rospy.Rate(1)
        goal_pose = Pose2D()
        tolerance = 0.5
        while not rospy.is_shutdown():
            if self.goal_publisher.get_num_connections() > 0:
                goal_pose.x = float(round(random.uniform(0.1, 10.0), 2))
                goal_pose.y = float(round(random.uniform(0.1, 10.0), 2))
                goal_pose.theta = float(round(random.uniform(-pi, pi), 2))
                self.goal_publisher.publish(goal_pose)

                while self.distanceToGoal(goal_pose) >= tolerance:
                    rospy.loginfo("Distance to goal = " + str(self.distanceToGoal(goal_pose)))
                    r.sleep()
            r.sleep()

    def distanceToGoal(self, goal_pose):
        return sqrt(pow((goal_pose.x - self.current_pose.pose.pose.position.x), 2) + pow((goal_pose.y - self.current_pose.pose.pose.position.y), 2))
