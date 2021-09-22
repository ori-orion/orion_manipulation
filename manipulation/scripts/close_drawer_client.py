#! /usr/bin/env python3
__author__ = "Mark Finean"
__email__ = "mfinean@robots.ox.ac.uk"

import rospy
import actionlib


from orion_actions.msg import *


def close_drawer_client():
    # Creates the SimpleActionClient, passing the type of the action
    client = actionlib.SimpleActionClient('close_drawer', CloseDrawerAction)

    print("Waiting for server")
    client.wait_for_server()
    print("Finished waiting for server")

    goal_msg = CloseDrawerGoal()
    client.send_goal(goal_msg)
    client.wait_for_result()

    return client.get_result()


if __name__ == '__main__':
    try:
        rospy.init_node('close_drawer_client')
        result = open_door_client()
        print("Result:" + str(result.result))
    except rospy.ROSInterruptException:
        print("Problem.")

