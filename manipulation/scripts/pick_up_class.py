#! /usr/bin/env python3
""" Base class for picking up objects.
"""
__author__ = "Mark Finean"
__email__ = "mfinean@robots.ox.ac.uk"

import hsrb_interface
import rospy
import actionlib
import numpy as np
import tf
import hsrb_interface.geometry as geometry
import tf.transformations as T
import math

from actionlib_msgs.msg import GoalStatus
from orion_actions.msg import *

from hsrb_interface import robot as _robot
_robot.enable_interactive()

# For grasp synthesis
from point_cloud_filtering.srv import SegmentObject
from gpd.msg import GraspConfigList

class PickUpBase(object):

    def __init__(self, name):
        # Preparation for using the robot functions
        self._action_name = None

        self.robot = hsrb_interface.Robot()
        self.whole_body = self.robot.try_get('whole_body')
        self.omni_base = self.robot.try_get('omni_base')
        self.collision_world = self.robot.try_get('global_collision_world')
        self.gripper = self.robot.try_get('gripper')
        self.whole_body.end_effector_frame = 'hand_palm_link'
        self.whole_body.looking_hand_constraint = True
        self.tts = self.robot.try_get('default_tts')
        self.tts.language = self.tts.ENGLISH

        self._CONNECTION_TIMEOUT = 15.0  # Define the vacuum timeouts
        self._SUCTION_TIMEOUT = rospy.Duration(20.0)  # Define the vacuum timeouts
        self._HAND_TF = 'hand_palm_link'
        self._GRASP_FORCE = 0.8
        self.whole_body.planning_timeout = 20.0 # Increase planning timeout. Default is 10s
        self.whole_body.tf_timeout = 10.0  # Increase tf timeout. Default is 5s

        self.grasps = None
        self.use_grasp_synthesis = None
        self.goal_object = None
        self._as = None

    def set_goal_object(self, obj):
        self.goal_object = obj

    def grasp_callback(self, msg):
        self.grasps = msg.grasps

    def get_grasp(self):

        while not len(self.grasps) > 0:
            if len(self.grasps) > 0:
                rospy.loginfo('Received %d grasps.', len(self.grasps))
                break

        grasp = self.grasps[0] # grasps are sorted in descending order by score
        rospy.loginfo('%s: Selected grasp with score:: %s' % (self._action_name, str(grasp.score)))

        # This gives the approach point correctly
        bottom = np.array([grasp.bottom.x, grasp.bottom.y, grasp.bottom.z])
        approach = np.array([grasp.approach.x, grasp.approach.y, grasp.approach.z ])
        binormal = np.array([grasp.binormal.x, grasp.binormal.y, grasp.binormal.z])
        hand_outer_diameter = 0.12
        hw = 0.5*hand_outer_diameter
        finger_width = 0.01
        left_bottom = bottom - (hw - 0.5 * finger_width) * binormal
        right_bottom = bottom + (hw - 0.5 * finger_width) * binormal
        base_center = left_bottom + 0.5 * (right_bottom - left_bottom) - 0.01 * approach
        approach_center = base_center - 0.06 * approach

        approach_4 = np.array([grasp.approach.x, grasp.approach.y, grasp.approach.z , approach_center[0]])
        binormal_4 = np.array([grasp.binormal.x, grasp.binormal.y, grasp.binormal.z, approach_center[1]])
        axis_4 = np.array([grasp.axis.x, grasp.axis.y, grasp.axis.z, approach_center[2]])

        R = np.array([axis_4, -binormal_4, approach_4, [0, 0, 0, 1]])
        q = T.quaternion_conjugate(T.quaternion_from_matrix(R))

        return geometry.Pose(geometry.Vector3(approach_center[0], approach_center[1], approach_center[2]), geometry.Quaternion(q[0], q[1], q[2], q[3]))

    def segment_object(self, object_pos_head_frame):
        rospy.wait_for_service('/object_segmentation')
        try:
            segment_object_service = rospy.ServiceProxy('/object_segmentation', SegmentObject)
            response = segment_object_service(object_pos_head_frame[0],
                                              object_pos_head_frame[1],
                                              object_pos_head_frame[2])
        except rospy.ServiceException as e:
            print( "Service call failed: %s" % e)

        return response

    def get_head_frame_object_pose(self, object_tf):
        found_trans = False
        listen = tf.TransformListener()
        rospy.sleep(3)
        while not found_trans:
            try:
                t = listen.getLatestCommonTime("/head_rgbd_sensor_rgb_frame", object_tf)
                (trans, rot) = listen.lookupTransform('/head_rgbd_sensor_rgb_frame', object_tf, t)
                found_trans = True
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                if self._as.is_preempt_requested():
                    rospy.loginfo('%s: Preempted. Moving to go and exiting.' % self._action_name)
                self.whole_body.move_to_go()
                self._as.set_preempted()
                continue

        return np.array([trans[0], trans[1], trans[2]])

    def get_goal_pose(self, relative=geometry.pose()):
        rospy.loginfo('%s: Trying to lookup goal pose...' % self._action_name)
        foundTrans = False
        while not foundTrans:
            try:
                odom_to_ref_pose = self.whole_body._lookup_odom_to_ref(self.goal_object)
                foundTrans = True
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                if self._as.is_preempt_requested():
                    rospy.loginfo('%s: Preempted. Moving to go and exiting.' % self._action_name)
                    self.whole_body.move_to_go()
                    self._as.set_preempted()
                    # TO DO - introudce a self.to_preempt = True
                    return

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
            if self._as.is_preempt_requested():
                rospy.loginfo('%s: Preempted. Moving to go and exiting.' % self._action_name)
                self.whole_body.move_to_go()
                self._as.set_preempted()
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
        rospy.sleep(3)
        while not found_trans:
            try:
                t = listen.getLatestCommonTime("/map", object_tf)
                (trans, rot) = listen.lookupTransform('/map', object_tf, t)
                found_trans = True
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                if self._as.is_preempt_requested():
                    rospy.loginfo('%s: Preempted. Moving to go and exiting.' % self._action_name)
                    self.whole_body.move_to_go()
                    self._as.set_preempted()
                    return

        return np.array([trans[0], trans[1], trans[2]])

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

        return math.sqrt(math.pow(trans[0],2) + math.pow(trans[1],2) + math.pow(trans[2],2))

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