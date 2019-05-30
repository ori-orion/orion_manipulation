#! /usr/bin/env python

import hsrb_interface
import rospy
import actionlib
import numpy as np
import tf
import hsrb_interface.geometry as geometry
import math

from hsrb_interface import robot as _robot
_robot.enable_interactive()

from actionlib_msgs.msg import GoalStatus
from orion_actions.msg import *


class PointToObjectAction(object):

    def __init__(self, name):
        self._action_name = 'point_to_object'
        self._as = actionlib.SimpleActionServer(self._action_name, 	orion_actions.msg.PointToObjectAction,
                                                execute_cb=self.execute_cb, auto_start=False)
        self._as.start()
        rospy.loginfo('%s: Action name is: %s' % (self._action_name, name))

        # Preparation for using the robot functions
        self.robot = hsrb_interface.Robot()
        self.whole_body = self.robot.try_get('whole_body')
        self.omni_base = self.robot.try_get('omni_base')
        self.gripper = self.robot.try_get('gripper')
        self.whole_body.end_effector_frame = 'hand_palm_link'
        self.tts = self.robot.try_get('default_tts')
        self.tts.language = self.tts.ENGLISH

        self._CONNECTION_TIMEOUT = 15.0 # Define the vacuum timeouts
        self._HAND_TF = 'hand_palm_link'
        self.whole_body.planning_timeout = 20.0 # Increase planning timeout. Default is 10s

        # Set up publisher for the collision map
        rospy.loginfo('%s: Initialised. Ready for clients.' % self._action_name)

    def check_for_object(self, object_tf):
        rospy.loginfo('%s: Checking object is in sight...' % self._action_name)
        found_marker = False
        listen = tf.TransformListener()
        rospy.sleep(1)
        while not found_marker:
            all_frames = listen.getFrameStrings()
            found_marker = object_tf in all_frames

    def get_similar_tf(self, tf_frame):
        listen = tf.TransformListener()
        rospy.sleep(3)
        all_frames = listen.getFrameStrings()
        for object_tf in all_frames:
            if tf_frame.split('_')[-1] in object_tf.split('-')[0]:
                return object_tf

    def get_object_pose(self, object_tf):
        found_trans = False
        listen = tf.TransformListener()
        rospy.sleep(1)
        while not found_trans:
            try:
                t = listen.getLatestCommonTime("/base_footprint", object_tf)
                (trans, rot) = listen.lookupTransform('/base_footprint', object_tf, t)
                found_trans = True
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue

        return np.array([trans[0], trans[1], trans[2]])

    def get_object_rel_to_hand(self, object_tf):
        found_trans = False
        listen = tf.TransformListener()
        rospy.sleep(1)
        while not found_trans:
            try:
                t = listen.getLatestCommonTime("/hand_palm_link", object_tf)
                (trans, rot) = listen.lookupTransform('/hand_palm_link', object_tf, t)
                found_trans = True
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue

        return np.array([trans[0], trans[1], trans[2]])

    def execute_cb(self, goal_msg):
        _result = PointToObjectResult()
        _result.result = False

        self.tts.say("I will close the gripper to make clear where I point.")
        rospy.sleep(1)
        rospy.loginfo('%s: Closing gripper to point at object' % self._action_name)
        self.gripper.set_distance(0.01)
        self.whole_body.move_to_neutral()

        goal_tf_in = goal_msg.object_tf_frame
        goal_tf = None

        while goal_tf is None:
            goal_tf = self.get_similar_tf(goal_tf_in)

            if self._as.is_preempt_requested():
                rospy.loginfo('%s: Preempted. Moving to go and exiting.' % self._action_name)
                self.whole_body.move_to_go()
                self._as.set_preempted()

            if goal_tf is None:
                rospy.loginfo('{0}: Found no similar tf frame. Trying again'.format(self._action_name))
        # Found the goal tf
        rospy.loginfo('{0}: Choosing tf frame "{1}".'.format(self._action_name, str(goal_tf)))

        self.tts.say("I can see the object and will now point towards it.")
        rospy.sleep(1)

        # ------------------------------------------------------------------------------
        # Check the object is in sight
        # self.check_for_object(goal_tf)

        # Look at the object - this is to make sure that we get all of the necessary collision map
        rospy.loginfo('%s: Moving head to look at the object.' % self._action_name)
        self.whole_body.gaze_point(ref_frame_id=goal_tf)

        object_pose = self.get_object_pose(goal_tf)
        theta = math.atan(object_pose[1] / object_pose[0])
        rospy.loginfo('%s: Turning to face the object.' % self._action_name)
        self.omni_base.go_rel(0, 0, theta)
        self.whole_body.gaze_point(ref_frame_id=goal_tf)

        object_rel_to_hand = self.get_object_rel_to_hand(goal_tf)
        distance = math.sqrt(math.pow(object_rel_to_hand[0], 2) + math.pow(object_rel_to_hand[1], 2) + math.pow(object_rel_to_hand[2], 2))
        if distance > 0.3:
            req_ratio = 0.3/distance
        else:
            req_ratio = 0.5

        rospy.loginfo('%s: Pointing to object.' % self._action_name)
        self.whole_body.linear_weight = 100
        self.whole_body.move_end_effector_pose(geometry.pose(x=req_ratio*object_rel_to_hand[0],
                                                             y=req_ratio*object_rel_to_hand[1],
                                                             z=req_ratio*object_rel_to_hand[2]),
                                               'hand_palm_link')

        self.tts.say("It is over there.")
        rospy.sleep(1)

        # Give opportunity to preempt
        if self._as.is_preempt_requested():
            rospy.loginfo('%s: Preempted. Moving to go and exiting.' % self._action_name)
            self.whole_body.move_to_go()
            self._as.set_preempted()
            return

        rospy.sleep(5)
        self.whole_body.move_to_go()

        rospy.loginfo('%s: Succeeded' % self._action_name)
        _result.result = True
        self._as.set_succeeded(_result)



if __name__ == '__main__':
    rospy.init_node('point_object_server_node')
    server = PointToObjectAction(rospy.get_name())
    rospy.spin()
