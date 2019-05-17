#! /usr/bin/env python

import rospy
import actionlib
from orion_actions.msg import *

def put_object_on_surface_client():
    client = actionlib.SimpleActionClient('put_object_on_surface', PutObjectOnSurfaceAction)

    print("Waiting for server")
    client.wait_for_server()
    print("Finished waiting for server")

    # Creates a goal to send to the action server.
    goal_msg = PutObjectOnSurfaceGoal()

    # Sends the goal to the action server.
    client.send_goal(goal_msg)

    # Waits for the server to finish performing the action.
    client.wait_for_result()

    # Prints out the result of executing the action
    return client.get_result()


if __name__ == '__main__':
    rospy.init_node('put_object_on_surface_client')
    result = put_object_on_surface_client()
    print("Result:" + str(result.result))


