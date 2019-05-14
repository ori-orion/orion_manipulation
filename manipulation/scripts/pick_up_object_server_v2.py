#! /usr/bin/env python

import hsrb_interface
import rospy
import actionlib
import numpy as np
import tf
import hsrb_interface.geometry as geometry
import tf.transformations as T
import math

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
        self.goal_pose_br = tf.TransformBroadcaster()

        self.goal_object = None
        rospy.loginfo('%s: Initialised. Ready for clients.' % self._action_name)

    def get_goal_pose(self, relative=geometry.pose()):
        rospy.loginfo('%s: Trying to lookup goal pose...' % self._action_name)
        foundTrans = False
        while not foundTrans:
            try:
                odom_to_ref_pose = self.whole_body._lookup_odom_to_ref(self.goal_object)
                foundTrans = True
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue
        odom_to_ref = geometry.pose_to_tuples(odom_to_ref_pose)
        odom_to_hand = geometry.multiply_tuples(odom_to_ref, relative)

        rospy.loginfo('%s: Calculated the goal pose.' % self._action_name)

        pose_msg = geometry.tuples_to_pose(odom_to_hand)

        eulers = T.euler_from_quaternion([pose_msg.orientation.x, pose_msg.orientation.y, pose_msg.orientation.z, pose_msg.orientation.w])

        return geometry.pose(x=pose_msg.position.x,
                             y=pose_msg.position.y,
                             z=pose_msg.position.z,
                             ei=eulers[0],
                             ej=eulers[1],
                             ek=eulers[2])


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

    def get_object_pose(self, object_tf):
        found_trans = False
        listen = tf.TransformListener()
        rospy.sleep(1)
        while not found_trans:
            try:
                t = listen.getLatestCommonTime("/map", object_tf)
                (trans, rot) = listen.lookupTransform('/map', object_tf, t)
                found_trans = True
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue

        return np.array([trans[0], trans[1], trans[2]])

    def set_goal_object(self, obj):
        self.goal_object = obj

    def callback(self, msg):
        self.pub.publish(msg)

    def grab_object(self, chosen_pregrasp_pose, chosen_grasp_pose):
        self.whole_body.end_effector_frame = 'hand_palm_link'

        try:
            rospy.loginfo('%s: Opening gripper.' % (self._action_name))
            self.gripper.command(1.2)

            # Move to pregrasp
            self.whole_body.collision_world = self.collision_world

            goal_pose = self.get_goal_pose(relative=chosen_pregrasp_pose)
            print goal_pose
            print type(goal_pose)
            self.goal_pose_br.sendTransform((goal_pose.pos.x, goal_pose.pos.y, goal_pose.pos.z),
                                            (goal_pose.ori.x, goal_pose.ori.y, goal_pose.ori.z, goal_pose.ori.w),
                                            rospy.Time.now(),
                                            'goal_pose',
                                            'base_footprint')

            rospy.loginfo('%s: Moving to pre-grasp position.' % (self._action_name))
            self.whole_body.move_end_effector_pose(goal_pose, 'base_footprint')

            # Turn off collision checking to get close and grasp
            rospy.loginfo('%s: Turning off collision checking to get closer.' % (self._action_name))
            self.whole_body.collision_world = None

            # Move to grasp pose
            rospy.loginfo('%s: Moving to grasp position.' % (self._action_name))
            self.whole_body.move_end_effector_pose(chosen_grasp_pose, self.whole_body.end_effector_frame)

            # Specify the force to grasp
            self.gripper.apply_force(self._GRASP_FORCE)
            rospy.loginfo('%s: Object grasped.' % self._action_name)

            return True

        except Exception as e:
            rospy.loginfo('{0}: Encountered exception {1}.'.format(self._action_name, str(e)))
            self.whole_body.collision_world = None
            rospy.loginfo('%s: Returning to neutral pose.' % (self._action_name))
            self.whole_body.move_to_neutral()
            self._as.set_aborted()
            return False

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
            rospy.loginfo('%s: Trying to move back and get into go position.' % self._action_name)
            self.omni_base.go_rel(-0.3, 0, 0)
            self.whole_body.move_to_go()
            return True
        except Exception as e:
            rospy.loginfo('{0}: Encountered exception {1}.'.format(self._action_name, str(e)))
            self.whole_body.collision_world = None
            try:
                rospy.loginfo('%s: Moving back and attempting to move to go again without collision detection.' % self._action_name)

                try:
                    self.omni_base.go_rel(-0.3, 0, 0)
                except:
                    rospy.loginfo('%s: Trying to move to the side instead.' % self._action_name)
                    try:
                        self.omni_base.go_rel(0, 0.3, 0)
                    except:
                        self.omni_base.go_rel(0, -0.3, 0)
                    self.whole_body.move_to_go()
            except:
                self.omni_base.go_rel(-0.3, 0, 0)
                self.whole_body.move_to_go()
            return False

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
            chosen_pregrasp_pose = geometry.pose(z=-0.05, ek=-math.pi/2)
            chosen_grasp_pose = geometry.pose(z=0.05)
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

        # Give opportunity to preempt
        if self._as.is_preempt_requested():
            rospy.loginfo('%s: Preempted. Moving to go and exiting.' % self._action_name)
            self.whole_body.move_to_go()
            self._as.set_preempted()
            return

        if grasp_type == 'suction':
            self.suck_object()
        else:
            grab_success = self.grab_object(chosen_pregrasp_pose, chosen_grasp_pose)

        if grab_success == False:
            self.whole_body.move_to_go()
            return

        # Now return to moving position
        success = self.finish_position()

        if success:
            rospy.loginfo('%s: Succeeded' % self._action_name)
            _result.result = True
            self._as.set_succeeded(_result)
        else:
            rospy.loginfo('%s: Failed' % self._action_name)
            _result.result = False
            self._as.set_aborted()


if __name__ == '__main__':
    rospy.init_node('pick_up_object_server_node')
    server = PickUpObjectAction(rospy.get_name())
    rospy.spin()