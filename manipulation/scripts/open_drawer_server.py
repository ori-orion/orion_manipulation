#! /usr/bin/env python
""" Action server for opening drawers.
Uses the /drawer_handle_detection service to do segmentation and find a centroid location for the handle to grasp.
If multiple drawer handles are detected, one is randomly chosen. Performs hard-coded pull back motion with
speech commentary
"""
import time
import hsrb_interface
import hsrb_interface.geometry as geometry
import numpy as np
import rospy
import actionlib

import tf
import tf.transformations as T
import math

from hsrb_interface import robot as _robot
__author__ = "Mark Finean"
__email__ = "mfinean@robots.ox.ac.uk"

_robot.enable_interactive()

from manipulation.manipulation_header import *
from orion_actions.msg import *
from point_cloud_filtering.srv import DetectDrawerHandles

class OpenDrawerAction(object):


    def __init__(self, name):
        self._action_name = 'open_drawer'
        self._as = actionlib.SimpleActionServer(self._action_name, 	orion_actions.msg.OpenDrawerAction,execute_cb=self.execute_cb, auto_start=False)
        self._as.start()

        # Preparation for using the robot functions
        self.robot = hsrb_interface.Robot()
        self.whole_body = self.robot.try_get('whole_body')
        self.omni_base = self.robot.try_get('omni_base')
        self.gripper = self.robot.try_get('gripper')
        self._HAND_TF = 'hand_palm_link'
        self._GRASP_FORCE = 0.8
        self.tts = self.robot.try_get('default_tts')
        self.tts.language = self.tts.ENGLISH

        # Increase planning timeout. Default is 10s
        self.whole_body.planning_timeout = 20.0

        rospy.loginfo('%s: Initialised. Ready for clients.' % (self._action_name))


    def get_handle_poses(self):
        rospy.wait_for_service('/drawer_handle_detection')
        try:
            detect_handle_service = rospy.ServiceProxy('/drawer_handle_detection', DetectDrawerHandles)
            response = detect_handle_service(True)
            # response should contain and array of x,y,z coords
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e

        return response

    def get_goal_pose(self, relative=geometry.pose(), offset=0.0):
        rospy.loginfo('%s: Trying to lookup goal pose...' % self._action_name)
        foundTrans = False
        while not foundTrans:
            try:
                odom_to_ref_pose = self.whole_body._lookup_odom_to_ref('head_rgbd_sensor_rgb_frame')
                foundTrans = True
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue
        odom_to_ref = geometry.pose_to_tuples(odom_to_ref_pose)
        odom_to_hand = geometry.multiply_tuples(odom_to_ref, relative)

        rospy.loginfo('%s: Calculated the goal pose.' % self._action_name)

        pose_msg = geometry.tuples_to_pose(odom_to_hand)

        # This gives the position of the handle
        goal_pose = geometry.pose(x=pose_msg.position.x,
                      y=pose_msg.position.y,
                      z=pose_msg.position.z,
                      ei=math.pi/2,
                      ej=0,
                      ek=math.pi/2)

        # return goal_pose
        offset_pose = geometry.pose(z=offset)
        #
        return geometry.multiply_tuples(goal_pose, offset_pose)

    def execute_cb(self, goal_msg):
        rospy.loginfo('%s: Executing callback. Opening drawers.' % (self._action_name))
        self.tts.say("I will now find handles and open the drawers.")
        rospy.sleep(1)

        rospy.loginfo('%s: Opening gripper.' % (self._action_name))
        self.gripper.command(1.2)


        # Try and find handles
        handle_poses = self.get_handle_poses()

        num_handles_found = len(handle_poses.x)

        rospy.loginfo('%s: Found %d handles.' % (self._action_name, num_handles_found))

        if num_handles_found > 0:
            self.tts.say("I found %d handles and will now grasp one of them" % num_handles_found)

            # Grasp the handle
            rospy.loginfo('%s: Grasping handle...' % (self._action_name))
            rospy.sleep(1)
            handle_goal_pose = self.get_goal_pose(relative=geometry.pose(x=handle_poses.x[0], y=handle_poses.y[0],
                                                                         z=handle_poses.z[0]), offset=-0.20)

            # self.whole_body.move_end_effector_pose(geometry.pose(x=handle_poses.x[0], y=handle_poses.y[0],  z=handle_poses.z[0]), 'head_rgbd_sensor_rgb_frame')
            self.whole_body.linear_weight = 80
            self.whole_body.move_end_effector_pose(handle_goal_pose, 'odom')
            self.whole_body.move_end_effector_pose(geometry.pose(z=0.055), 'hand_palm_link')

            # try:
            #     self.whole_body.move_end_effector_pose(geometry.pose(z=0.02), 'hand_palm_link')
            # except:
            #     rospy.loginfo("%s: Couldn't move forward..." % (self._action_name))
            #     pass
            # self.gripper.set_distance(self._GRASP_FORCE)
            self.gripper.apply_force(self._GRASP_FORCE)
            rospy.sleep(2)

            rospy.loginfo('%s: Executing opening motion...' % (self._action_name))
            self.tts.say("Grasped successfully. Now opening.")
            rospy.sleep(1)

            # Open motion
            self.whole_body.planning_timeout = 5.0
            try:
                rospy.loginfo('%s: Attempting to move backwards...' % (self._action_name))
                self.whole_body.linear_weight = 1
                self.whole_body.move_end_effector_pose(geometry.pose(z=-0.1), 'hand_palm_link')
                rospy.loginfo('%s: Attempting to move back more...' % (self._action_name))
                self.whole_body.move_end_effector_pose(geometry.pose(z=-0.1), 'hand_palm_link')
                self.tts.say("Releasing door handle.")
                rospy.sleep(1)
                self.gripper.set_distance(0.1)
                self.whole_body.move_end_effector_pose(geometry.pose(z=-0.1), 'hand_palm_link')
                self.whole_body.move_to_go()
            except:
                rospy.loginfo('%s: Problem moving backwards...' % (self._action_name))
                self.tts.say("Sorry. I encountered a problem.")
                rospy.sleep(1)
                self.gripper.set_distance(0.1)
                self.whole_body.move_end_effector_pose(geometry.pose(z=-0.1), 'hand_palm_link')
                self.whole_body.move_to_go()
                self._as.set_aborted()
        else:
            rospy.loginfo('%s: No handles found.' % (self._action_name))
            self.tts.say("Sorry. I cannot see any handles to grasp.")
            rospy.sleep(1)
            self._as.set_aborted()

        result = OpenDrawerResult()
        result.result = True
        self._as.set_succeeded(result)


if __name__ == '__main__':
    rospy.init_node('open_drawer_server_node')
    server = OpenDrawerAction(rospy.get_name())
    rospy.spin()
