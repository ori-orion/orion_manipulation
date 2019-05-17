#! /usr/bin/env python

import rospy
import actionlib


from orion_actions.msg import *


def open_drawer_client():
    # Creates the SimpleActionClient, passing the type of the action
    client = actionlib.SimpleActionClient('open_drawer', OpenDrawerAction)

    print("Waiting for server")
    client.wait_for_server()
    print("Finished waiting for server")

    goal_msg = OpenDrawerGoal()
    client.send_goal(goal_msg)
    client.wait_for_result()

    return client.get_result()


if __name__ == '__main__':
    try:
        rospy.init_node('open_drawer_client')
        result = open_drawer_client()
        print("Result:" + str(result.result))
    except rospy.ROSInterruptException:
        print("Problem.")

