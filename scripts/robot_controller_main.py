#!/usr/bin/env python

import exceptions
from ros_robot_controller import RobotController

if __name__ == '__main__':
    try:
        robot_controller = RobotController()
    except Exception as e:
        print "Robot controller node is running out of scope."