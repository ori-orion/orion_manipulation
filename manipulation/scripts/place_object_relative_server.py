#! /usr/bin/env python3
""" Action server for placing an object relative to another.
Currently in development.
"""

__author__ = "Mark Finean"
__email__ = "mfinean@robots.ox.ac.uk"

import hsrb_interface
import rospy
import actionlib
import hsrb_interface.geometry as geometry

from manipulation.manipulation_header import CollisionMapper
from actionlib_msgs.msg import GoalStatus
from tmc_manipulation_msgs.msg import CollisionObject
from orion_actions.msg import *


class PlaceObjectRelativeAction(object):

    def __init__(self, name):
        self._action_name = 'place_object_relative'
        self._as = actionlib.SimpleActionServer(self._action_name, 	orion_actions.msg.PlaceObjectRelativeAction,
                                                execute_cb=self.execute_cb, auto_start=False)
        self._as.start()
        rospy.loginfo('%s: Action name is: %s' % (self._action_name, name))

        # Preparation for using the robot functions
        self.robot = hsrb_interface.Robot()
        self.whole_body = self.robot.try_get('whole_body')
        self.collision_world = self.robot.try_get('global_collision_world')
        self.gripper = self.robot.try_get('gripper')
        self.whole_body.end_effector_frame = 'hand_palm_link'
        self.whole_body.looking_hand_constraint = True

        self.collision_mapper = CollisionMapper(self.robot)

        self.whole_body.planning_timeout = 20.0 # Increase planning timeout. Default is 10s

        # Set up publisher for the collision map
        self.pub = rospy.Publisher('known_object', CollisionObject, queue_size=1)

        rospy.loginfo('%s: Initialised. Ready for clients.' % self._action_name)

    def callback(self, msg):
        self.pub.publish(msg)

    def place_object_relative(self, object_tf, pose):
        try:
            self.whole_body.collision_world = self.collision_world

            rospy.loginfo('%s: Placing gripper close to floor in front' % (self._action_name))
            self.whole_body.move_end_effector_pose(pose, object_tf)

            rospy.loginfo('%s: Opening gripper.' % (self._action_name))
            self.gripper.command(1.2)

        except Exception as e:
            rospy.loginfo('{0}: Encountered exception {1}.'.format(self._action_name, str(e)))
            self.whole_body.collision_world = None
            rospy.loginfo('%s: Returning to neutral pose.' % (self._action_name))
            self.whole_body.move_to_neutral()
            self._as.set_aborted()

    def execute_cb(self, goal_msg):

        # Unpack the goal tf and relative position desired
        goal_tf = goal_msg.goal_tf
        # goal_msg.y, goal_msg.z
        rel_pose = geometry.pose(z=goal_msg.x)

        _result = PlaceObjectRelativeResult()
        _result.result = False

        # Currently doesn't do anything other than relay to another topic
        rospy.Subscriber("known_object_pre_filter", CollisionObject, self.callback)

        # Set collision map
        rospy.loginfo('%s: Getting Collision Map.' % self._action_name)
        self.collision_mapper.get_collision_map()
        rospy.loginfo('%s: Collision Map generated.' % self._action_name)

        # Give opportunity to preempt
        if self._as.is_preempt_requested():
            rospy.loginfo('%s: Preempted' % self._action_name)
            self._as.set_preempted()
            return

        self.place_object_relative(goal_tf, rel_pose)

        rospy.loginfo('%s: Returning to neutral pose.' % (self._action_name))
        self.whole_body.move_to_neutral()

        rospy.loginfo('%s: Succeeded' % self._action_name)
        _result.result = True
        self._as.set_succeeded(_result)


if __name__ == '__main__':
    rospy.init_node('place_object_relative_action_server_node')
    server = PlaceObjectRelativeAction(rospy.get_name())
    rospy.spin()
