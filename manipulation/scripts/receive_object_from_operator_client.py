#! /usr/bin/env python3
""" Client for receive_object_from_operator action.
"""

import rospy
import actionlib
import orion_actions.msg as msg


def receive_object_from_operator_client():
    client = actionlib.SimpleActionClient('receive_object_from_operator', msg.ReceiveObjectFromOperatorAction)

    print("Waiting for server")
    client.wait_for_server()
    print("Finished waiting for server")

    # Creates a goal to send to the action server.
    goal_msg = msg.ReceiveObjectFromOperatorGoal()

    # Sends the goal to the action server.
    client.send_goal(goal_msg)

    # Waits for the server to finish performing the action.
    client.wait_for_result()

    # Return the result of executing the action
    return client.get_result()


if __name__ == '__main__':
    rospy.init_node('receive_object_from_operator_client')

    result = receive_object_from_operator_client()
    print("Result:" + str(result.result))
