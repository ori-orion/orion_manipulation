#! /usr/bin/env python

import rospy
import actionlib
import numpy as np
import tf
import math

import hsrb_interface
import hsrb_interface.geometry as geometry

from std_srvs.srv import Empty, EmptyRequest
# from move_base_msgs.msg import *
from move_base_msgs.msg import MoveBaseAction
from move_base_msgs.msg import MoveBaseGoal
from actionlib_msgs.msg import GoalStatus
from orion_actions.msg import *
from geometry_msgs.msg import PoseStamped

from hsrb_interface import robot as _robot
_robot.enable_interactive()

class FollowAction(object):

    def __init__(self, name):
        self._action_name = 'follow'
        self._as = actionlib.SimpleActionServer(self._action_name, 	orion_actions.msg.FollowAction,
                                                execute_cb=self.execute_cb, auto_start=False)
        self._as.start()

        self.pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)

        self.robot = hsrb_interface.Robot()
        self.whole_body = self.robot.try_get('whole_body')
        self.whole_body.end_effector_frame = 'hand_palm_link'
        self.whole_body.looking_hand_constraint = True
        self.omni_base = self.robot.try_get('omni_base')
        self.tts = self.robot.try_get('default_tts')
        self.tts.language = self.tts.ENGLISH

        rospy.loginfo('%s: Action name is: %s' % (self._action_name, name))
        rospy.loginfo('%s: Initialised. Ready for clients.' % self._action_name)


    def get_object_pose(self, object_tf):
        found_trans = False
        listen = tf.TransformListener()
        while not found_trans:
            try:
                t = listen.getLatestCommonTime("/base_footprint", object_tf)
                (trans, rot) = listen.lookupTransform('/base_footprint', object_tf, t)
                found_trans = True
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException, tf.Exception):
                # rospy.loginfo('%s: Cant find object pose. Trying again....' % self._action_name)
                if self._as.is_preempt_requested():
                    rospy.loginfo('%s: Preempted. Moving to go and exiting.' % self._action_name)
                    self.whole_body.move_to_go()
                    self._as.set_preempted()
                    return None

        return np.array([trans[0], trans[1]])

    def check_for_object(self, object_tf):
        rospy.loginfo('%s: Checking object is in sight...' % self._action_name)
        found_marker = False
        listen = tf.TransformListener()
        rospy.sleep(1)
        while not found_marker:
            all_frames = listen.getFrameStrings()
            found_marker = object_tf in all_frames
            if self._as.is_preempt_requested():
                rospy.loginfo('%s: Preempted. Moving to go and exiting.' % self._action_name)
                self.whole_body.move_to_go()
                self._as.set_preempted()
                return True

    def get_similar_tf(self, tf_frame):
        listen = tf.TransformListener()
        rospy.sleep(3)
        all_frames = listen.getFrameStrings()
        for object_tf in all_frames:
            if tf_frame.split('_')[-1] in object_tf.split('-')[0]:
                return object_tf

    def execute_cb(self, goal_msg):
        _result = FollowResult()
        _result.succeeded = False

        goal_tf = None

        rospy.loginfo('{0}: Finding similar tf frame.'.format(self._action_name))

        is_preempted = False

        while goal_tf is None:
            goal_tf = self.get_similar_tf(goal_msg.object_name)

            if goal_tf is not None:
                rospy.loginfo('{0}: Choosing tf frame "{1}".'.format(self._action_name, str(goal_tf)))

            # if goal_tf is None:
            #     # rospy.loginfo('{0}: Found no similar tf frame.'.format(self._action_name))
            #     pass
            # else:
            #     rospy.loginfo('{0}: Choosing tf frame "{1}".'.format(self._action_name, str(goal_tf)))

            # Give opportunity to preempt
            if self._as.is_preempt_requested():
                rospy.loginfo('%s: Preempted. Moving to go and exiting.' % self._action_name)
                self.whole_body.move_to_go()
                self._as.set_preempted()
                is_preempted = True
                return

        if is_preempted:
            return

        self.tts.say("I will now start following. Please do not go too fast.")
        rospy.sleep(1)

        while True:
            # Check the object is in sight
            is_preempted = self.check_for_object(goal_tf)
            if is_preempted:
                return

            rospy.loginfo('%s: Moving head to look at the object.' % self._action_name)
            can_look = False
            while not can_look:
                try:
                    self.whole_body.gaze_point(ref_frame_id=goal_tf)
                    can_look = True
                except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException, tf.Exception):
                    rospy.loginfo('%s: Preempted. Moving to go and exiting.' % self._action_name)
                    if self._as.is_preempt_requested():
                        rospy.loginfo('%s: Preempted. Moving to go and exiting.' % self._action_name)
                        self.whole_body.move_to_go()
                        self._as.set_preempted()
                        is_preempted = True
                        return

            if is_preempted:
                return

            rospy.loginfo('%s: Getting person pose.' % self._action_name)
            person_coords = self.get_object_pose(goal_tf)

            if person_coords is None:
                return

            rospy.loginfo('{0}: Found the person pose.'.format(self._action_name))
            distance = math.sqrt(math.pow(person_coords[1], 2) + math.pow(person_coords[0], 2))
            theta = math.atan(person_coords[1] / person_coords[0])
            rospy.loginfo('{0}: Distance to person: "{1} Theta: {2}".'.format(self._action_name,
                                                                              str(distance),
                                                                              str(theta)))


            # Stop head moving
            stop_client = rospy.ServiceProxy('/viewpoint_controller/stop', Empty)
            stop_client.call(EmptyRequest())

            if distance <= 0.5:
                rospy.loginfo('%s: Sending base goals.' % self._action_name)
                # self.omni_base.go_rel(0, 0, theta)
                base_pose = geometry.pose(0, 0, 0.0, 0.0, 0.0, theta)
                base_goal = self.omni_base.create_go_pose_goal(base_pose, 'base_footprint')
                self.omni_base.execute(base_goal)
                rospy.loginfo('%s: Base movement complete. Continuing to follow.' % self._action_name)
            else:
                ratio = 1 - (0.5/distance)
                rospy.loginfo('%s: Sending base goals.' % self._action_name)
                # self.omni_base.go_rel(ratio * person_coords[0], ratio * person_coords[1], theta)
                base_pose = geometry.pose(ratio * person_coords[0], ratio * person_coords[1], 0.0, 0.0, 0.0, theta)
                base_goal = self.omni_base.create_go_pose_goal(base_pose, 'base_footprint')
                self.omni_base.execute(base_goal)

                rospy.loginfo('%s: Base movement complete. Continuing to follow.' % self._action_name)

        # Give opportunity to preempt
            if self._as.is_preempt_requested():
                rospy.loginfo('%s: Preempted. Moving to go and exiting.' % self._action_name)
                self.tts.say("I will now stop following you.")
                rospy.sleep(1)
                start_client = rospy.ServiceProxy('/viewpoint_controller/start', Empty)
                start_client.call(EmptyRequest())
                self.whole_body.move_to_go()
                self._as.set_preempted()
                is_preempted = True
                return

            if is_preempted:
                return


if __name__ == '__main__':
    rospy.init_node('follow_server_node')
    server = FollowAction(rospy.get_name())
    rospy.spin()
