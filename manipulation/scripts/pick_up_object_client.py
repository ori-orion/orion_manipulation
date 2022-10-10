#! /usr/bin/env python3
""" Client for pick_up_object action.
"""

import rospy
import sys
import argparse
import actionlib
import orion_actions.msg as msg


def pick_up_object_client(goal_tf, approach_axis=None):
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

def arg_parser():
    parser = argparse.ArgumentParser(description ='sort some integers.')

    parser.add_argument('goal_tf', type = str)
    parser.add_argument('--approach_axis', type = str)

    return parser

def str_to_tuple(txt):
    assert txt[0] == '(' and txt[-1] == ')'
    split_txt = txt[1: -1].split(',')

    return tuple([float(item.strip()) for item in split_txt])

if __name__ == "__main__":
    rospy.init_node("pick_up_object_client")

    # args = arg_parser().parse_args()
    #
    # goal_tf = args.goal_tf.strip()
    # approach_axis = args.approach_axis

    # if approach_axis is not None:
    #     approach_axis = str_to_tuple(approach_axis)

    if len(sys.argv) == 2:
        goal_tf = sys.argv[1]
    else:
        print("Failed to provide tf frame as argument - defaulting to ar_marker/201")
        goal_tf = "ar_marker/201"

    # print(approach_axis)
    print(goal_tf)

    result = pick_up_object_client(goal_tf)
    print("Result:" + str(result.result))
