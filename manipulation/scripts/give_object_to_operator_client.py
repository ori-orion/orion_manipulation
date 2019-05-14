#! /usr/bin/env python

import rospy
import actionlib

from orion_actions.msg import *


def give_object_to_operator_client():

    client = actionlib.SimpleActionClient('give_something', GiveObjectToOperatorAction)
    client.wait_for_server()
    goal_msg = GiveObjectToOperatorGoal()
    client.send_goal(goal_msg)
    client.wait_for_result()
    return client.get_result()


if __name__ == '__main__':
    try:
        rospy.init_node('give_object_to_operator_client_py')
        result = give_object_to_operator_client()
        print("Result:" + str(result.result))
    except rospy.ROSInterruptException:
        print("Problem.")

