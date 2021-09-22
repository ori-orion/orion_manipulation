#! /usr/bin/env python3
__author__ = "Mark Finean"
__email__ = "mfinean@robots.ox.ac.uk"

import rospy
# from __future__ import print_function
import sys
# Brings in the SimpleActionClient
import actionlib

# Brings in the messages used by the fibonacci action, including the
# goal message and the result message.

from manipulation.msg import *

def move_hand_to_tf_client(goal_tf):
    # Creates the SimpleActionClient, passing the type of the action
    # (FibonacciAction) to the constructor.
    client = actionlib.SimpleActionClient('move_hand_to_tf_server_node', MoveHandToTfAction)

    # Waits until the action server has started up and started
    # listening for goals.
    client.wait_for_server()

    # Creates a goal to send to the action server.
    goal_msg = MoveHandToTfGoal(goal_tf=goal_tf)

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
        rospy.init_node('move_hand_to_tf_client_py')
        goal_tf = 'ar_marker/4000'
        result = move_hand_to_tf_client(goal_tf)
        print("Result:", ', '.join([str(n) for n in result.sequence]))
    except rospy.ROSInterruptException:
        print("Problem.")
        # print("program interrupted before completion", file=sys.stderr)
