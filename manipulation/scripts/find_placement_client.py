#! /usr/bin/env python3
""" Client for find_placement action.
"""

import rospy
import sys
import argparse
from manipulation.srv import FindPlacement

def find_placement_client(goal_tf, goal_pos, dims, maxHeight, radius, candidateNum, zShift):
    print("Waiting for server")
    rospy.wait_for_service('find_placement_around')
    print("Finished waiting for server")
    try:
        find_placement = rospy.ServiceProxy('find_placement_around', FindPlacement)
        resp = find_placement(goal_tf, goal_pos, dims, maxHeight, radius, zShift, candidateNum)
        return resp.position
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def arg_parser():
    parser = argparse.ArgumentParser(description ='sort some integers.')

    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument('--goal_tf', type=str, nargs='?', default="")
    group.add_argument('--goal_pos', type=str, nargs='?', default="")

    parser.add_argument('-d', '--dims', type=str, default="(0.1, 0.1, 0.1)")
    parser.add_argument('-m', '--maxHeight', type=float, default=0.1)
    parser.add_argument('-r', '--radius', type=float, default=0.1)
    parser.add_argument('-n', '--candidateNum', type=int, default=8)
    parser.add_argument('-z', '--zShift', type=float, default=0.05)

    return parser

def str_to_tuple(txt):
    txt = txt.strip()
    assert txt[0] == '(' and txt[-1] == ')'
    split_txt = txt[1: -1].split(',')

    return tuple([float(item.strip()) for item in split_txt])

if __name__ == "__main__":
    rospy.init_node("find_placement_client")

    args = arg_parser().parse_args()

    goal_tf = ''
    goal_pos = ()

    if args.goal_tf == "" and args.goal_pos == "":
        print("Failed to provide tf frame or position as argument - defaulting to ar_marker/201")
        goal_tf = "ar_marker/201"
    else:
        if args.goal_tf != "":
            goal_tf = args.goal_tf.strip()
        if args.goal_pos != "":
            goal_pos = str_to_tuple(args.goal_pos)

    dims = str_to_tuple(args.dims)
    h = args.maxHeight
    r = args.radius
    n = args.candidateNum
    z = args.zShift

    result = find_placement_client(goal_tf, goal_pos, dims, h, r, n, z)
    print("Result:" + str(result))