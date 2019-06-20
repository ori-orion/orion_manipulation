#! /usr/bin/env python
__author__ = "Mark Finean"
__email__ = "mfinean@robots.ox.ac.uk"

import rospy
import actionlib

from orion_actions.msg import *


def receive_object_from_operator_client():

    client = actionlib.SimpleActionClient('receive_object_from_operator', ReceiveObjectFromOperatorAction)
    client.wait_for_server()
    goal_msg = ReceiveObjectFromOperatorGoal()
    client.send_goal(goal_msg)
    client.wait_for_result()
    return client.get_result()


if __name__ == '__main__':
    try:
        rospy.init_node('receive_object_from_operator_client')
        result = receive_object_from_operator_client()
        print("Result:" + str(result.result))
    except rospy.ROSInterruptException:
        print("Problem.")

