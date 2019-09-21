#!/usr/bin/env python

import exceptions
from ros_GopiGo_line_follower import GoPiGoLineFollower

if __name__ == '__main__':
    try:
        lineFollower = GoPiGoLineFollower()
    except Exception as e:
        print "Robot controller node is running out of scope."