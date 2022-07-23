#! /usr/bin/env python3
""" Client for pick_up_bin_bag action.
"""

import rospy
import actionlib
import orion_actions.msg as msg


def pick_up_bin_bag_client():
    client = actionlib.SimpleActionClient("pick_up_bin_bag", msg.PickUpBinBagAction)

    print("Waiting for server")
    client.wait_for_server()
    print("Finished waiting for server")

    # Creates a goal to send to the action server.
    goal_msg = msg.PickUpBinBagGoal()

    # Sends the goal to the action server.
    client.send_goal(goal_msg)

    # Waits for the server to finish performing the action.
    client.wait_for_result()

    # Return the result of executing the action
    return client.get_result()


if __name__ == "__main__":
    rospy.init_node("pick_up_bin_bag_client")

    result = pick_up_bin_bag_client()
    print("Result:" + str(result.result))
