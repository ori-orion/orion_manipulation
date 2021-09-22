#! /usr/bin/env python3

import rospy
import actionlib
from orion_actions.msg import *

def pick_up_bin_bag_client():
    client = actionlib.SimpleActionClient('pick_up_bin_bag', PickUpBinBagAction)

    print("Waiting for server")
    client.wait_for_server()
    print("Finished waiting for server")

    # Creates a message to send to the action server.
    msg = PickUpBinBagGoal()

    # Sends the message to the action server.
    client.send_goal(msg)

    # Waits for the server to finish performing the action.
    client.wait_for_result()

    # Prints out the result of executing the action
    return client.get_result()


if __name__ == '__main__':
    rospy.init_node('pick_up_bin_bag_client')

    result = pick_up_bin_bag_client()
    print("Result:" + str(result.result))
