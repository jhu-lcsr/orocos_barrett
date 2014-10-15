#!/usr/bin/env python

from __future__ import print_function

import rospy

import actionlib
from oro_barrett_msgs.msg import BHandGraspAction, BHandGraspGoal

def main():
    rospy.init_node('grasp_cli')

    grasp_client = actionlib.SimpleActionClient(
        'grasp', BHandGraspAction)

    grasp_client.wait_for_server()

    goal = BHandGraspGoal()
    goal.grasp_mask = [True, True, True]
    goal.grasp_speed = [2.0, 2.0, 2.0]
    goal.grasp_effort = [1.0, 1.0, 1.0]

    grasp_client.send_goal(goal)
    grasp_client.wait_for_result()

    print(grasp_client.get_result())

    return 0


if __name__ == '__main__':
    main()
