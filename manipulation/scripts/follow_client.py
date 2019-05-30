#! /usr/bin/env python

import rospy
import sys
import actionlib
from orion_actions.msg import *

def follow_client(goal_tf):
    client = actionlib.SimpleActionClient('follow', FollowAction)

    print("Waiting for server")
    client.wait_for_server()
    print("Finished waiting for server")

    # Creates a goal to send to the action server.
    goal_msg = FollowGoal(object_name=goal_tf)

    # Sends the goal to the action server.
    client.send_goal(goal_msg)

    # Waits for the server to finish performing the action.
    client.wait_for_result()

    # Prints out the result of executing the action
    return client.get_result()


if __name__ == '__main__':
    rospy.init_node('follow_client_py')
    print("Provided {0} arguments to client.".format(len(sys.argv)))

    if len(sys.argv) == 2:
        goal_tf = sys.argv[1]
    else:
        print("Failed to provide tf frame as argument to pick up. Defaulting to ar_marker/201")
        goal_tf = 'ar_marker/201'
        # sys.exit(1)

    result = follow_client(goal_tf)
    print("Result:" + str(result.result))


