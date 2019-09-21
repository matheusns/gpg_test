#!/usr/bin/env python

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
        self.pose_listener = rospy.Subscriber("gpg/mobile_base_controller/odom", Odometry, self.callbackPose)
        self.goal_listener = rospy.Subscriber("gpg/goal", Pose2D, self.callbackGoal)
        self.velocity_publisher = rospy.Publisher('gpg/mobile_base_controller/cmd_vel', Twist, queue_size=10)

    def initMemberVariables(self):
        self.current_pose = Odometry()

    def callbackPose(self, msg):
        self.current_pose = msg

    def callbackGoal(self, msg):
        current_goal = msg
        print ("here 1")
        self.goToGoal(current_goal)

    def goToGoal(self, goal_position):
        rospy.loginfo("Moving robot to the pose: (" + str(goal_position.x) + " , " + str(
            goal_position.y) + ")" + " and theta = " + str(goal_position.theta))

        vel_msg = Twist()

        vel_msg.linear.y = 0
        vel_msg.linear.z = 0

        vel_msg.angular.x = 0
        vel_msg.angular.y = 0

        print ("here 2")

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
        return sqrt(pow((goal_pose.x - x), 2) + pow((goal_pose.y - y), 2))

    def linearVelocity(self, goal_pose, gain=0.1):
        return gain * self.euclideanDistance(goal_pose)

    def angularVelocity(self, goal_pose, gain=0.1):
        return gain * ros_angles.shortest_angular_distance(self.yawFromQuaternion(self.current_pose.pose.pose.orientation), self.angleToTheGoal(goal_pose))

    def yawFromQuaternion(self, quaternionMsg):

        quaternionList = self.quaternionMsg2List(quaternionMsg)
        euler = tf.transformations.euler_from_quaternion(quaternionList)
        return euler[2]

    def quaternionMsg2List(self, quaternionMsg):
        quaternion = (quaternionMsg.x, quaternionMsg.y, quaternionMsg.z, quaternionMsg.w)
        return quaternion

    def angleToTheGoal(self, goal_pose):
        return atan2(goal_pose.y - y, goal_pose.x - x)
