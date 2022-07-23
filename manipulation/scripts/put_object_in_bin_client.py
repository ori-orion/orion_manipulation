#! /usr/bin/env python3
""" Client for put_object_in_bin action.
"""

import rospy
import actionlib
import orion_actions.msg as msg


def put_object_in_bin_client():
    client = actionlib.SimpleActionClient('put_object_in_bin', msg.PutObjectInBinAction)

    print("Waiting for server")
    client.wait_for_server()
    print("Finished waiting for server")

    # Creates a goal to send to the action server.
    goal_msg = msg.PutObjectInBinGoal()

    # Sends the goal to the action server.
    client.send_goal(goal_msg)

    # Waits for the server to finish performing the action.
    client.wait_for_result()

    # Return the result of executing the action
    return client.get_result()


if __name__ == '__main__':
    rospy.init_node('put_object_in_bin_client')

    result = put_object_in_bin_client()
    print("Result:" + str(result.result))
