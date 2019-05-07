#! /usr/bin/env python

import rospy
import actionlib
from orion_actions.msg import *

def place_object_relative_client(goal_tf):
    client = actionlib.SimpleActionClient('pick_up_object_server_node', PlaceObjectRelativeAction)

    print("Waiting for server")
    client.wait_for_server()
    print("Finished waiting for server")

    # Creates a goal to send to the action server.
    goal_msg = PlaceObjectRelativeGoal(goal_tf=goal_tf, x=-0.1, y=0, z=0)

    # Sends the goal to the action server.
    client.send_goal(goal_msg)

    # Waits for the server to finish performing the action.
    client.wait_for_result()

    # Prints out the result of executing the action
    return client.get_result()


if __name__ == '__main__':
    rospy.init_node('place_object_relative_client_py')
    goal_tf = 'ar_marker/4000'
    result = place_object_relative_client(goal_tf)
    print("Result:" + str(result.result))


