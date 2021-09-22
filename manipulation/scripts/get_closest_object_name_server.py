#! /usr/bin/env python3
""" Action server for getting name of the closest object in tf.
"""
__author__ = "Mark Finean"
__email__ = "mfinean@robots.ox.ac.uk"

import rospy
import actionlib
import numpy as np
import tf
import tf.transformations as T
import math

from actionlib_msgs.msg import GoalStatus
from orion_actions.msg import *


class GetClosestObjectNameAction(object):

    def __init__(self, name):
        self._action_name = 'get_closest_object_name'
        self._as = actionlib.SimpleActionServer(self._action_name, 	orion_actions.msg.GetClosestObjectNameAction,
                                                execute_cb=self.execute_cb, auto_start=False)
        self._as.start()

        self.non_pickable_objects = ['chair', 'table', 'person', 'refrigerator', 'microwave', 'cupboard', 'television', 'sink']

        rospy.loginfo('%s: Action name is: %s' % (self._action_name, name))
        rospy.loginfo('%s: Initialised. Ready for clients.' % self._action_name)


    def get_tf_objects(self):
        listen = tf.TransformListener()
        rospy.sleep(3)
        all_frames = listen.getFrameStrings()

        return all_frames


    def get_object_distance(self, object_tf):
        found_trans = False
        listen = tf.TransformListener()
        rospy.sleep(1)
        while not found_trans:
            try:
                t = listen.getLatestCommonTime("/hand_palm_link", object_tf)
                (trans, rot) = listen.lookupTransform('/hand_palm_link', object_tf, t)
                found_trans = True
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                if self._as.is_preempt_requested():
                    rospy.loginfo('%s: Preempted. Moving to go and exiting.' % self._action_name)
                    self.whole_body.move_to_go()
                    self._as.set_preempted()
                    return

        return math.sqrt(math.pow(trans[0], 2) + math.pow(trans[1], 2) + math.pow(trans[2], 2))

    def execute_cb(self, goal_msg):
        _result = GetClosestObjectNameResult()

        closest_object = None
        closest_object_distance = 1000

        potential_objects = self.get_tf_objects()

        for object in potential_objects:

            if object in self.non_pickable_objects:
                continue

            dist = self.get_object_distance(object)
            if dist < closest_object_distance:
                closest_object_distance = dist
                closest_object = object

        rospy.loginfo('{0}: Closest object is {1}.'.format(self._action_name, closest_object))
        rospy.loginfo('{0}: Distance to object is "{1:.2f}"m.'.format(self._action_name, closest_object_distance))

        _result.object = closest_object
        self._as.set_succeeded(_result)

if __name__ == '__main__':
    rospy.init_node('get_closest_object_name_server_node')
    server = GetClosestObjectNameAction(rospy.get_name())
    rospy.spin()
