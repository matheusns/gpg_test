#!/usr/bin/env python

# Launch file created to attend the requirements established on the Ex4 by the discipline of Intelligent Control
# of Robotics Systems
# Professor: Wouter Caarls
# Students: Matheus do Nascimento Santos 1920858 (@matheusns)
#           Luciana Reys 1920856 (@lsnreys)

import exceptions
from ros_robot_controller import RobotController

if __name__ == '__main__':
    try:
        robot_controller = RobotController()
    except Exception as e:
        print "Robot controller node is running out of scope."