#! /usr/bin/env python

import rospy
import actionlib


from orion_actions.msg import *


def open_door_client():
    # Creates the SimpleActionClient, passing the type of the action
    client = actionlib.SimpleActionClient('open_door', OpenDoorAction)

    print("Waiting for server")
    client.wait_for_server()
    print("Finished waiting for server")

    goal_msg = OpenDoorGoal()
    client.send_goal(goal_msg)
    client.wait_for_result()

    return client.get_result()


if __name__ == '__main__':
    try:
        rospy.init_node('open_door_client_py')
        result = open_door_client()
        print("Result:" + str(result.result))
    except rospy.ROSInterruptException:
        print("Problem.")

