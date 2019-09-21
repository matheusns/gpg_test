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

import angles as ros_angles
from geometry_msgs.msg import Twist

import rospy
import tf
from gpg_remote.msg import State

from nav_msgs.msg import Odometry

from math import pow, atan2, sqrt

import time
import sys, select, termios, tty

class GoPiGoLineFollower:
    def __init__(self):
        rospy.init_node('GoPiGo_line_follower_node')
        self.initMemberVariables()
        self.initROSChannels()
        self.followLine()

    def initMemberVariables(self):
        # Enables/Disables debug mode
        self.debug = True
        self.gpgEnable = True

        # # Position to take action on
        self.mid = [0, 0, 1, 0, 0]       # Middle Position.
        self.mid1 = [0, 1, 1, 1, 0]      # Middle Position.
        self.small_l = [0, 1, 1, 0, 0]   # Slightly to the left.
        self.small_l1 = [0, 1, 0, 0, 0]  # Slightly to the left.
        self.small_r = [0, 0, 1, 1, 0]   # Slightly to the right.
        self.small_r1 = [0, 0, 0, 1, 0]  # Slightly to the right.
        self.left = [1, 1, 0, 0, 0]      # Sensor reads strongly to the left.
        self.left1 = [1, 0, 0, 0, 0]     # Sensor reads strongly to the left.
        self.right = [0, 0, 0, 1, 1]     # Sensor reads strongly to the right.
        self.right1 = [0, 0, 0, 0, 1]    # Sensor reads strongly to the right.
        self.stop = [1, 1, 1, 1, 1]      # Sensor reads stop.
        self.stop1 = [0, 0, 0, 0, 0]     # Sensor reads stop.

        # ################# Weird states ##################
        self.weird_state_1 = [1, 0, 1, 1, 1]

        self.currentSensorSet = []

        self.current_pose = Odometry()

        # [x,y,z,theta]
        # todo: Update gains and increments
        self.directions = {}
        self.directions["goStraight"] = [1, 0, 0, 0]
        self.directions["turnSlightLeft"] = [1,0,0,1]
        self.directions["turnLeft"] = [1, 0, 0, 1]
        self.directions["turnSlightRight"] = [1,0,0,-1]
        self.directions["turnRight"] = [0,0,0,-1]
        self.directions["stop"] = [0, 0, 0, 0]
        self.directions["goBack"] = [-1, 0, 0, 0]

    def initROSChannels(self):
        self.sensors_listener = rospy.Subscriber("gpg/state", State, self.callbackState)
        self.velocity_publisher = rospy.Publisher('gpg/mobile_base_controller/cmd_vel', Twist, queue_size=10)


    def callbackState(self, msg):
        self.line_sensors = msg.line
        self.currentSensorSet = self.lineInt2binary(self.line_sensors)

    def lineInt2binary(self, line_int_values):
        l = []
        output = l
        for lineValue in line_int_values:
            if lineValue < 960:
                output.append(0)
            else:
                output.append(1)
        return output

    def followLine(self):
        rate = rospy.Rate(20)

        while not rospy.is_shutdown():
            # white line reached
            if self.currentSensorSet == self.stop:
                if self.debug:
                    rospy.loginfo("It has reached the Finish.")
            else:
                if self.sensors_listener.get_num_connections() > 0:
                    if self.debug:
                        print str(self.currentSensorSet)
                    time.sleep(0.01)
                    self.moveGpg()
            rate.sleep()

    def moveGpg(self):
        # if the line is in the middle, keep moving straight
        # if the line is slightly left of right, keep moving straight
        if (self.currentSensorSet == self.small_r or self.currentSensorSet == self.small_l or \
                self.currentSensorSet == self.mid or self.currentSensorSet == self.mid1):
            self.go_straight()
        # If the line is towards the sligh left, turn slight right
        elif self.currentSensorSet == self.small_l1:
            self.turn_slight_right()
        elif self.currentSensorSet == self.left or self.currentSensorSet == self.left1:
            self.turn_right()
        # If the line is towards the sligh right, turn slight left
        elif self.currentSensorSet == self.small_r1:
            self.turn_slight_left()
        elif self.currentSensorSet == self.right or self.currentSensorSet == self.right1:
            self.turn_left()
        elif self.currentSensorSet == self.stop1:
            self.go_back()

        time.sleep(0.01)

    def go_straight(self):
        if self.debug:
            print("Going straight")
        if self.gpgEnable:
            self.velocityController("goStraight")

    def turn_slight_left(self):
        if self.debug:
            print("Turn slight left")
        if self.gpgEnable:
            self.velocityController("turnSlightLeft")

    def turn_left(self):
        if self.debug:
            print("Turn left")
        if self.gpgEnable:
            self.velocityController("turnLeft")

    def turn_slight_right(self):
        if self.debug:
            print("Turn slight right")
        if self.gpgEnable:
            self.velocityController("turnSlightRight")

    def turn_right(self):
        if self.debug:
            print("Turn right")
        if self.gpgEnable:
            self.velocityController("turnRight")

    def stop_now(self):
        if self.debug:
            print("Stop")
        if self.gpgEnable:
            self.velocityController("stop")

    def go_back(self):
        if self.debug:
            print("Go Back")
        if self.gpgEnable:
            self.velocityController("goBack")

    def velocityController(self, desired_direction):
        speed = 0.05
        turn = 0.2

        y = 0
        z = 0
        x = 0
        th = 0

        x = self.directions[desired_direction][0]
        y = self.directions[desired_direction][1]
        z = self.directions[desired_direction][2]
        th = self.directions[desired_direction][3]

        vel_msg = Twist()
        vel_msg.linear.x = x*speed; vel_msg.linear.y = y*speed; vel_msg.linear.z = z*speed;
        vel_msg.angular.x = 0; vel_msg.angular.y = 0; vel_msg.angular.z = th * turn

        self.velocity_publisher.publish(vel_msg)
