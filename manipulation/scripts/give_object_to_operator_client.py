#! /usr/bin/env python3
""" Client for give_object_to_operator action.
"""

import rospy
import actionlib
from orion_actions.msg import GiveObjectToOperatorAction, GiveObjectToOperatorGoal


def give_object_to_operator_client():
    client = actionlib.SimpleActionClient('give_object_to_operator', GiveObjectToOperatorAction)

    print("Waiting for server")
    client.wait_for_server()
    print("Finished waiting for server")

    # Creates a goal to send to the action server.
    goal_msg = GiveObjectToOperatorGoal()

    # Sends the goal to the action server.
    client.send_goal(goal_msg)

    # Waits for the server to finish performing the action.
    client.wait_for_result()

    # Return the result of executing the action
    return client.get_result()


if __name__ == '__main__':
    rospy.init_node('give_object_to_operator_client_py')

    result = give_object_to_operator_client()
    print("Result:" + str(result.result))
