#! /usr/bin/env python3
""" Client for pick_up_object action.
"""

import rospy
import sys
import actionlib
import orion_actions.msg as msg


def pick_up_object_client(goal_tf):
    client = actionlib.SimpleActionClient("pick_up_object", msg.PickUpObjectAction)

    print("Waiting for server")
    client.wait_for_server()
    print("Finished waiting for server")

    # Creates a goal to send to the action server.
    goal_msg = msg.PickUpObjectGoal(goal_tf=goal_tf)

    # Sends the goal to the action server.
    client.send_goal(goal_msg)

    # Waits for the server to finish performing the action.
    client.wait_for_result()

    # Return the result of executing the action
    return client.get_result()


if __name__ == "__main__":
    rospy.init_node("pick_up_object_client")

    if len(sys.argv) == 2:
        goal_tf = sys.argv[1]
    else:
        print("Failed to provide tf frame as argument - defaulting to ar_marker/201")
        goal_tf = "ar_marker/201"

    result = pick_up_object_client(goal_tf)
    print("Result:" + str(result.result))
