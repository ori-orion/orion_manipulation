#! /usr/bin/env python

import rospy
# from __future__ import print_function
import sys
# Brings in the SimpleActionClient
import actionlib

# Brings in the messages used by the fibonacci action, including the
# goal message and the result message.

from manipulation.msg import *

def give_object_to_operator_client():
    # Creates the SimpleActionClient, passing the type of the action
    # (FibonacciAction) to the constructor.
    client = actionlib.SimpleActionClient('give_object_to_operator_server_node', GiveObjectToOperatorAction)

    # Waits until the action server has started up and started
    # listening for goals.
    client.wait_for_server()

    # Creates a goal to send to the action server.
    goal_msg = GiveObjectToOperatorGoal()

    # Sends the goal to the action server.
    client.send_goal(goal_msg)

    # Waits for the server to finish performing the action.
    client.wait_for_result()

    # Prints out the result of executing the action
    return client.get_result()  # A FibonacciResult

if __name__ == '__main__':
    try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        rospy.init_node('give_object_to_operator_client_py')
	result = give_object_to_operator_client()

        print("Result:", ', '.join([str(n) for n in result.sequence]))
    except rospy.ROSInterruptException:
        print("Problem.")

