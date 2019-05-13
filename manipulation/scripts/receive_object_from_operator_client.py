#! /usr/bin/env python

import rospy
import actionlib

from orion_actions.msg import *


def receive_object_from_operator_client():

    client = actionlib.SimpleActionClient('receive_object_from_operator_server_node', ReceiveObjectFromOperatorAction)
    client.wait_for_server()
    goal_msg = ReceiveObjectFromOperatorGoal()
    client.send_goal(goal_msg)
    client.wait_for_result()
    return client.get_result()


if __name__ == '__main__':
    try:
        rospy.init_node('receive_object_from_operator_client_py')
        result = receive_object_from_operator_client()
        print("Result:" + str(result.result))
    except rospy.ROSInterruptException:
        print("Problem.")

