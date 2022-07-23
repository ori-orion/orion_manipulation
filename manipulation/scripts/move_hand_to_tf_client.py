#! /usr/bin/env python3
""" Client for move_hand_to_tf action.
"""

import rospy
import sys
import actionlib
import orion_actions.msg as msg


def move_hand_to_tf_client(goal_tf):
    client = actionlib.SimpleActionClient("move_hand_to_tf", msg.MoveHandToTfAction)

    print("Waiting for server")
    client.wait_for_server()
    print("Finished waiting for server")

    # Creates a goal to send to the action server.
    goal_msg = msg.MoveHandToTfGoal(goal_tf=goal_tf)

    # Sends the goal to the action server.
    client.send_goal(goal_msg)

    # Waits for the server to finish performing the action.
    client.wait_for_result()

    # Return the result of executing the action
    return client.get_result()


if __name__ == "__main__":
    rospy.init_node("move_hand_to_tf_client")

    if len(sys.argv) == 2:
        goal_tf = sys.argv[1]
    else:
        print("Failed to provide tf frame as argument - defaulting to ar_marker/201")
        goal_tf = "ar_marker/201"

    result = move_hand_to_tf_client(goal_tf)
    print("Result:" + str(result.result))
