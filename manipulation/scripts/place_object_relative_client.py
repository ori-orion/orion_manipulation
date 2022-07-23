#! /usr/bin/env python3
""" Client for place_object_relative action.
"""

import rospy
import sys
import actionlib
import orion_actions.msg as msg


def place_object_relative_client(goal_tf, x, y, z):
    client = actionlib.SimpleActionClient("place_object_relative", msg.PlaceObjectRelativeAction)

    print("Waiting for server")
    client.wait_for_server()
    print("Finished waiting for server")

    # Creates a goal to send to the action server.
    goal_msg = msg.PlaceObjectRelativeGoal(goal_tf=goal_tf, x=x, y=y, z=z)

    # Sends the goal to the action server.
    client.send_goal(goal_msg)

    # Waits for the server to finish performing the action.
    client.wait_for_result()

    # Return the result of executing the action
    return client.get_result()


if __name__ == "__main__":
    rospy.init_node("place_object_relative_client")

    if len(sys.argv) == 2:
        goal_tf = sys.argv[1]
    else:
        print("Failed to provide tf frame as argument - defaulting to ar_marker/201")
        goal_tf = "ar_marker/201"

    x = y = z = 0.1

    result = place_object_relative_client(goal_tf, x, y, z)
    print("Result:" + str(result.result))
