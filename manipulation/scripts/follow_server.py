#! /usr/bin/env python

import rospy
import actionlib
import numpy as np
import tf
import math
from move_base_msgs.msg import *
from actionlib_msgs.msg import GoalStatus
from orion_actions.msg import *
from geometry_msgs.msg import PoseStamped

# def movebase_client(x, y, theta):
#
#     client = actionlib.SimpleActionClient('move_base_simple', MoveBaseAction)
#     client.wait_for_server()
#
#     goal = MoveBaseGoal()
#     goal.target_pose.header.frame_id = "/base_footprint"
#     goal.target_pose.header.stamp = rospy.Time.now()
#     goal.target_pose.pose.position.x = x
#     goal.target_pose.pose.position.y = y
#     goal.target_pose.pose.orientation.w = theta
#
#     client.send_goal(goal)
#     wait = client.wait_for_result()
#
#     if not wait:
#         rospy.logerr("Action server not available!")
#         rospy.signal_shutdown("Action server not available!")
#     else:
#         return client.get_result()


def get_object_pose(object_tf):
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

    return np.array([trans[0], trans[1]])

class FollowAction(object):

    def __init__(self, name):
        self._action_name = 'follow'
        self._as = actionlib.SimpleActionServer(self._action_name, 	orion_actions.msg.FollowAction,
                                                execute_cb=self.execute_cb, auto_start=False)
        self._as.start()

        self.pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)

        rospy.loginfo('%s: Action name is: %s' % (self._action_name, name))
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
            rospy.loginfo('%s: Found tf frame: %s' % (self._action_name, object_tf))
            if tf_frame.split('_')[-1] in object_tf.split('-')[0]:
                return object_tf

    def send_movebase_goal(self, x, y, theta):

        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = "/base_footprint"
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0

        quaternion = tf.transformations.quaternion_from_euler(0, 0, theta)
        pose.pose.orientation.x = quaternion[0]
        pose.pose.orientation.y = quaternion[1]
        pose.pose.orientation.z = quaternion[2]
        pose.pose.orientation.w = quaternion[3]

        self.pub.publish(pose)

    def execute_cb(self, goal_msg):
        _result = FollowResult()
        _result.succeeded = False

        goal_tf = None

        while goal_tf is None:
            goal_tf = self.get_similar_tf(goal_msg.object_name)
            if goal_tf is None:
                rospy.loginfo('{0}: Found no similar tf frame. Aborting.'.format(self._action_name))
            else:
                rospy.loginfo('{0}: Choosing tf frame "{1}".'.format(self._action_name, str(goal_tf)))

            # Give opportunity to preempt
            if self._as.is_preempt_requested():
                rospy.loginfo('%s: Preempted. Moving to go and exiting.' % self._action_name)
                self.whole_body.move_to_go()
                self._as.set_preempted()
                rospy.loginfo('%s: Succeeded' % self._action_name)
                _result.succeeded = True
                self._as.set_succeeded(_result)
                return

        while True:
            # Check the object is in sight
            self.check_for_object(goal_tf)

            person_coords = get_object_pose(goal_tf)

            distance = math.sqrt(math.pow(person_coords[1], 2) + math.pow(person_coords[0], 2))
            theta = math.atan(person_coords[1] / person_coords[0])

            self.send_movebase_goal(0, 0, theta)
            rospy.sleep(1)
            if distance > 0.5:
                self.send_movebase_goal(distance-0.5, 0, 0)
            rospy.sleep(1)

            # Give opportunity to preempt
            if self._as.is_preempt_requested():
                rospy.loginfo('%s: Preempted. Moving to go and exiting.' % self._action_name)
                self.whole_body.move_to_go()
                self._as.set_preempted()
                rospy.loginfo('%s: Succeeded' % self._action_name)
                _result.succeeded = True
                self._as.set_succeeded(_result)
                return



if __name__ == '__main__':
    rospy.init_node('follow_server_node')
    server = FollowAction(rospy.get_name())
    rospy.spin()
