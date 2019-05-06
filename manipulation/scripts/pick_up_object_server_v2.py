#! /usr/bin/env python

import hsrb_interface
import rospy
import actionlib
import numpy as np
import tf
import hsrb_interface.geometry as geometry

from manipulation.manipulation_header import CollisionMapper
from actionlib_msgs.msg import GoalStatus
from tmc_suction.msg import (SuctionControlAction, SuctionControlGoal)
from tmc_manipulation_msgs.msg import CollisionObject
from orion_actions.msg import *


class PickUpObjectAction(object):

    def __init__(self, name):
        self._action_name = 'pick_up_object'
        self._as = actionlib.SimpleActionServer(self._action_name, 	orion_actions.msg.PickUpObjectAction,
                                                execute_cb=self.execute_cb, auto_start=False)
        self._as.start()
        rospy.loginfo('%s: Action name is: %s' % (self._action_name, name))

        # Preparation for using the robot functions
        self.robot = hsrb_interface.Robot()
        self.whole_body = self.robot.try_get('whole_body')
        self.omni_base = self.robot.try_get('omni_base')
        self.collision_world = self.robot.try_get('global_collision_world')
        self.gripper = self.robot.try_get('gripper')
        self.whole_body.end_effector_frame = 'hand_palm_link'
        self.whole_body.looking_hand_constraint = True

        self.collision_mapper = CollisionMapper(self.robot)

        self._CONNECTION_TIMEOUT = 15.0 # Define the vacuum timeouts
        self._SUCTION_TIMEOUT = rospy.Duration(20.0) # Define the vacuum timeouts
        self._HAND_TF = 'hand_palm_link'
        self._GRASP_FORCE = 0.8
        self.whole_body.planning_timeout = 20.0 # Increase planning timeout. Default is 10s

        # Set up publisher for the collision map
        self.pub = rospy.Publisher('known_object', CollisionObject, queue_size=1)
        self.tf_listener = tf.TransformListener()
        self.goal_object = None
        rospy.loginfo('%s: Initialised. Ready for clients.' % self._action_name)

    def check_for_object(self, object_tf):
        rospy.loginfo('%s: Checking object is in sight...' % self._action_name)
        found_marker = False
        while not found_marker:
            all_frames = self.tf_listener.getFrameStrings()
            found_marker = object_tf in all_frames

    def get_similar_tf(self, tf_frame):
        rospy.sleep(3)
        all_frames = self.tf_listener.getFrameStrings()
        for object_tf in all_frames:
            if tf_frame.split('_')[-1] in object_tf.split('-')[0]:
                return object_tf

    def get_object_pose(self, object_tf):
        found_trans = False
        while not found_trans:
            try:
                t = self.tf_listener.getLatestCommonTime("/map", object_tf)
                (trans, rot) = self.tf_listener.lookupTransform('/map', object_tf, t)
                found_trans = True
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue

        return np.array([trans[0], trans[1], trans[2]])

    def set_goal_object(self, obj):
        self.goal_object = obj

    def callback(self, msg):
        self.pub.publish(msg)

    def grab_object(self, chosen_pregrasp_pose, chosen_grasp_pose):
        try:
            rospy.loginfo('%s: Opening gripper.' % (self._action_name))
            self.gripper.command(1.2)

            # Move to pregrasp
            self.whole_body.collision_world = self.collision_world
            rospy.loginfo('%s: Moving to pre-grasp position.' % (self._action_name))
            self.whole_body.move_end_effector_pose(chosen_pregrasp_pose, self.goal_object)

            # Turn off collision checking to get close and grasp
            rospy.loginfo('%s: Turning off collision checking to get closer.' % (self._action_name))
            self.whole_body.collision_world = None

            # Move to grasp pose
            rospy.loginfo('%s: Moving to grasp position.' % (self._action_name))
            self.whole_body.move_end_effector_pose(chosen_grasp_pose, self.whole_body.end_effector_frame)

            # Specify the force to grasp
            self.gripper.apply_force(self._GRASP_FORCE)
            rospy.loginfo('%s: Object grasped.' % self._action_name)

        except Exception as e:
            rospy.loginfo('{0}: Encountered exception {1}.'.format(self._action_name, str(e)))
            self.whole_body.collision_world = None
            rospy.loginfo('%s: Returning to neutral pose.' % (self._action_name))
            self.whole_body.move_to_neutral()
            self._as.set_aborted()

    def suck_object(self):
        self.whole_body.end_effector_frame = 'hand_l_finger_vacuum_frame'

        rospy.loginfo('%s: Closing gripper.' % self._action_name)
        self.gripper.command(0.1)

        rospy.loginfo('%s: Turning on on the suction...' % self._action_name)

        # Create action client to control suction
        suction_action = '/hsrb/suction_control'
        suction_control_client = actionlib.SimpleActionClient(suction_action, SuctionControlAction)

        # Wait for connection
        try:
            if not suction_control_client.wait_for_server(rospy.Duration(self._CONNECTION_TIMEOUT)):
                raise Exception(suction_action + ' does not exist')
        except Exception as e:
            rospy.logerr(e)

        # Send a goal to start suction
        rospy.loginfo('%s: Suction server found. Activating suction...' % self._action_name)
        suction_on_goal = SuctionControlGoal()
        suction_on_goal.timeout = self._SUCTION_TIMEOUT
        suction_on_goal.suction_on.data = True

        if (suction_control_client.send_goal_and_wait(suction_on_goal) == GoalStatus.SUCCEEDED):
            rospy.loginfo('Suction succeeded. Object picked up')
        else:
            rospy.loginfo('Suction failed')
            return

        self.whole_body.collision_world = self.collision_world
        self.whole_body.move_end_effector_pose(geometry.pose(z=0.05, ei=3.14), self.goal_object)

        # Turn off collision checking to get close and grasp
        rospy.loginfo('%s: Turning off collision checking to get closer.' % (self._action_name))
        self.whole_body.collision_world = None
        self.whole_body.move_end_effector_pose(geometry.pose(z=0.045), self.goal_object)

    def finish_position(self):
        self.whole_body.collision_world = self.collision_world

        try:
            rospy.loginfo('%s: Trying to move to go.' % self._action_name)
            self.omni_base.go_rel(-0.3, 0, 0)
            self.whole_body.move_to_go()
        except Exception as e:
            rospy.loginfo('{0}: Encountered exception {1}.'.format(self._action_name, str(e)))
            try:
                rospy.loginfo('%s: Moving back and attempting to move to go again without collision detection.' % self._action_name)
                try:
                    self.omni_base.go_rel(-0.3, 0, 0)
                except:
                    try:
                        self.omni_base.go_rel(0, 0.3, 0)
                    except:
                        self.omni_base.go_rel(0, -0.3, 0)
                        self.whole_body.move_to_go()
            except:
                self.whole_body.collision_world = None
                self.omni_base.go_rel(-0.3, 0, 0)
                self.whole_body.move_to_go()

    def execute_cb(self, goal_msg):
        _result = PickUpObjectResult()
        _result.result = False

        # Currently doesn't do anything other than relay to another topic
        rospy.Subscriber("known_object_pre_filter", CollisionObject, self.callback)

        goal_tf = goal_msg.goal_tf
        goal_tf = self.get_similar_tf(goal_tf)
        if goal_tf is None:
            self._as.set_aborted()
            rospy.loginfo('{0}: Found no similar tf frame. Aborting.'.format(self._action_name))
            return

        rospy.loginfo('{0}: Choosing tf frame "{1}".'.format(self._action_name, str(goal_tf)))
        self.set_goal_object(goal_tf)

        if self.goal_object == 'postcard':
            grasp_type = 'suction'
        else:
            grasp_type = 'grab'
            chosen_pregrasp_pose = geometry.pose(z=-0.05, ei=0)
            chosen_grasp_pose = geometry.pose(z=0.03)
        # ------------------------------------------------------------------------------
        # Check the object is in sight
        self.check_for_object(self.goal_object)

        # Look at the object - this is to make sure that we get all of the necessary collision map
        rospy.loginfo('%s: Moving head to look at the object.' % self._action_name)
        self.whole_body.gaze_point(ref_frame_id=self.goal_object)

        # Set collision map
        rospy.loginfo('%s: Getting Collision Map.' % self._action_name)
        self.collision_mapper.get_collision_map()
        rospy.loginfo('%s: Collision Map generated.' % self._action_name)

        # Get the object pose to subtract from collision map
        self.check_for_object(goal_tf)

        # rospy.loginfo('%s: Getting object pose.' % self._action_name)
        # goal_object_pose = self.get_object_pose(self.goal_object)

        # Give opportunity to preempt
        if self._as.is_preempt_requested():
            rospy.loginfo('%s: Preempted' % self._action_name)
            self._as.set_preempted()
            return

        if grasp_type == 'suction':
            self.suck_object()
        else:
            self.grab_object(chosen_pregrasp_pose, chosen_grasp_pose)

        # Now return to moving position
        self.finish_position()

        rospy.loginfo('%s: Succeeded' % self._action_name)
        _result.result = True
        self._as.set_succeeded(_result)


if __name__ == '__main__':
    rospy.init_node('pick_up_object_server_node')
    server = PickUpObjectAction(rospy.get_name())
    rospy.spin()
