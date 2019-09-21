#!/usr/bin/env python

import angles as ros_angles
from geometry_msgs.msg import Twist

import rospy
import tf
from gpg_remote.msg import State

import time
from nav_msgs.msg import Odometry

from math import pow, atan2, sqrt

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

        self.currentSensorSet = []

        self.current_pose = Odometry()

        # [kl, ka, position(x,y,theta) + desired increments]
        # todo: Update gains and increments
        self.directions = {}
        self.directions["goStraight"] = [1, 1, 0.1, 0.2, 0.1]
        self.directions["turnSlightLeft"] = [1, 1, 0.1, 0.2, 0.6 ]
        self.directions["turnLeft"] = [1, 1, 0.1, 0.2, 02]
        self.directions["turnSlightRight"] = [1, 1, 0.01, 0.02, 0.6]
        self.directions["turnRight"] = [1, 1, 0.1, 0.2, 0.6]
        self.directions["stop"] = [1, 1, 0.1, 0.2, 0.6]
        self.directions["goBack"] = [1, 1, 0.1, 0.2, 0.6]

    def initROSChannels(self):
        self.sensors_listener = rospy.Subscriber("gpg/state", State, self.callbackState)
        self.pose_listener = rospy.Subscriber("gpg/mobile_base_controller/odom", Odometry, self.callbackPose)
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

    def callbackPose(self, msg):
        self.current_pose = msg
        self.curr_x = self.current_pose.pose.pose.position.x
        self.curr_y = self.current_pose.pose.pose.position.y
        self.curr_theta = self.yawFromQuaternion(self.current_pose.pose.pose.orientation)


    def followLine(self):
        rate = rospy.Rate(20)

        while not rospy.is_shutdown():
            # white line reached
            print str(self.currentSensorSet)

            if self.currentSensorSet == self.stop1 or self.currentSensorSet == self.stop:
                if self.debug:
                    rospy.loginfo("Out of line boundaries, stopping.")
            else:
                if self.pose_listener.get_num_connections() > 0:
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
        elif self.currentSensorSet == self.stop:
            self.stop_now()

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
        vel_msg = Twist()

        vel_msg.linear.y = 0
        vel_msg.linear.z = 0

        vel_msg.angular.x = 0
        vel_msg.angular.y = 0

        distance_tolerance = 0.2

        rate = rospy.Rate(20)

        increment_x = self.directions[desired_direction][2]
        increment_y = self.directions[desired_direction][3]
        increment_theta = self.directions[desired_direction][4]

        linear_gain = self.directions[desired_direction][0]
        angular_gain = self.directions[desired_direction][1]


        goal_position = (self.curr_x + increment_x, self.curr_y + increment_y, self.curr_theta + increment_theta)
        # print "Angular gain = " + str(angular_gain)
        print "Goal  = " + str(goal_position)

        while self.euclideanDistance(goal_position) >= distance_tolerance:
            vel_msg.linear.x = self.linearVelocity(goal_position, linear_gain)
            vel_msg.angular.z = self.angularVelocity(goal_position, angular_gain)

            self.velocity_publisher.publish(vel_msg)
            rate.sleep()

        # todo: Check if removing it is valid
        vel_msg.linear.x = 0
        vel_msg.angular.z = 0


        self.velocity_publisher.publish(vel_msg)

    def euclideanDistance(self, goal_pose):
        return sqrt(pow((goal_pose[0] - self.curr_x), 2) + pow((goal_pose[1] - self.curr_y), 2))

    def linearVelocity(self, goal_pose, gain=1):
        return gain * self.euclideanDistance(goal_pose)

    def angularVelocity(self, goal_pose, gain=4):
        return gain * ros_angles.shortest_angular_distance(self.curr_theta, self.angleToTheGoal(goal_pose))

    def angleToTheGoal(self, goal_pose):
        return atan2(goal_pose[1] - self.curr_y, goal_pose[0] - self.curr_x)

    def yawFromQuaternion(self, quaternionMsg):
        quaternionList = self.quaternionMsg2List(quaternionMsg)
        euler = tf.transformations.euler_from_quaternion(quaternionList)
        return euler[2]

    def quaternionMsg2List(self, quaternionMsg):
        quaternion = (quaternionMsg.x, quaternionMsg.y, quaternionMsg.z, quaternionMsg.w)
        return quaternion