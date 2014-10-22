#!/usr/bin/env python

from __future__ import print_function

import sys
import rospy

import actionlib
from oro_barrett_msgs.msg import BHandSpreadAction, BHandSpreadGoal

def main():
    rospy.init_node('spread_cli')

    my_argv = rospy.myargv(argv=sys.argv)

    spread_client = actionlib.SimpleActionClient(
        'spread', BHandSpreadAction)

    spread_client.wait_for_server()

    goal = BHandSpreadGoal()
    goal.spread_position = float(my_argv[1]) if (len(my_argv) > 1) else 0.0

    spread_client.send_goal(goal)
    spread_client.wait_for_result()

    print(spread_client.get_result())

    return 0


if __name__ == '__main__':
    main()
