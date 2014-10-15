#!/usr/bin/env python

from __future__ import print_function

import rospy

import actionlib
from oro_barrett_msgs.msg import BHandReleaseAction, BHandReleaseGoal

def main():
    rospy.init_node('release_cli')

    release_client = actionlib.SimpleActionClient(
        'release', BHandReleaseAction)

    release_client.wait_for_server()

    goal = BHandReleaseGoal()
    goal.release_mask = [True, True, True]
    goal.release_speed = [3.0, 3.0, 3.0]

    release_client.send_goal(goal)
    release_client.wait_for_result()

    print(release_client.get_result())

    return 0


if __name__ == '__main__':
    main()

