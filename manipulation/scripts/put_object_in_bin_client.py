#! /usr/bin/env python3

import rospy
import actionlib
from orion_actions.msg import *

def put_object_in_bin_client():
    client = actionlib.SimpleActionClient('put_object_in_bin', PutObjectInBinAction)

    print("Waiting for server")
    client.wait_for_server()
    print("Finished waiting for server")

    # Creates a message to send to the action server.
    msg = PutObjectInBinGoal()

    # Sends the message to the action server.
    client.send_goal(msg)

    # Waits for the server to finish performing the action.
    client.wait_for_result()

    # Prints out the result of executing the action
    return client.get_result()


if __name__ == '__main__':
    rospy.init_node('put_object_in_bin_client_node')

    result = put_object_in_bin_client()
    print("Result:" + str(result.result))
