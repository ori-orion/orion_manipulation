#! /usr/bin/env python3
""" Client for find_placement action.
"""

import rospy
import sys
import argparse
from manipulation.srv import FindPlacement

def find_placement_client(goal_tf, dims, maxHeight, radius, candidateNum):
    print("Waiting for server")
    rospy.wait_for_service('find_placement_around')
    print("Finished waiting for server")
    try:
        find_placement = rospy.ServiceProxy('find_placement_around', FindPlacement)
        resp = find_placement(goal_tf, dims, maxHeight, radius, candidateNum)
        return resp.position
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def arg_parser():
    parser = argparse.ArgumentParser(description ='sort some integers.')

    parser.add_argument('goal_tf', type=str, nargs='?', default="")
    parser.add_argument('-d', '--dims', type=str, default="(0,1, 0.1, 0.1)")
    parser.add_argument('-m', '--maxHeight', type=float, default=0.1)
    parser.add_argument('-r', '--radius', type=float, default=0.1)
    parser.add_argument('-n', '--candidateNum', type=int, default=8)

    return parser

def str_to_tuple(txt):
    assert txt[0] == '(' and txt[-1] == ')'
    split_txt = txt[1: -1].split(',')

    return tuple([float(item.strip()) for item in split_txt])

if __name__ == "__main__":
    rospy.init_node("find_placement_client")

    args = arg_parser().parse_args()

    goal_tf = args.goal_tf.strip()
    dims = str_to_tuple(args.dims)
    h = args.maxHeight
    r = args.radius
    n = args.candidateNum

    if goal_tf == "":
        print("Failed to provide tf frame as argument - defaulting to ar_marker/201")
        goal_tf = "ar_marker/201"

    result = find_placement_client(goal_tf, dims, h, r, n)
    print("Result:" + str(result))