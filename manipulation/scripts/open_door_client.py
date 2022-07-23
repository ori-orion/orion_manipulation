#! /usr/bin/env python3
""" Client for open_door action.
"""

import rospy
import actionlib
import orion_actions.msg as msg


def open_door_client():
    client = actionlib.SimpleActionClient('open_door', msg.OpenDoorAction)

    print("Waiting for server")
    client.wait_for_server()
    print("Finished waiting for server")

    # Creates a goal to send to the action server.
    goal_msg = msg.OpenDoorGoal()

    # Sends the goal to the action server.
    client.send_goal(goal_msg)

    # Waits for the server to finish performing the action.
    client.wait_for_result()

    # Return the result of executing the action
    return client.get_result()


if __name__ == '__main__':
    rospy.init_node('open_door_client')

    result = open_door_client()
    print("Result:" + str(result.result))
