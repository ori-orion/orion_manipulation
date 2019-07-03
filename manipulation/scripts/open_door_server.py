#! /usr/bin/env python
""" Action server for opening a door.
Uses the /handle_detection service to do segmentation and find a centroid location for the handle to grasp.
Robust to left and right hinged doors. Currently works on pull down horizontal handles and executes a pull open motion.
"""
__author__ = "Mark Finean"
__email__ = "mfinean@robots.ox.ac.uk"

import time
import hsrb_interface
import hsrb_interface.geometry as geometry
import numpy as np
import rospy
import actionlib
import json
import rospkg
import tf
import tf2_ros
import geometry_msgs.msg
import math
from hsrb_interface import robot as _robot

_robot.enable_interactive()

from manipulation.manipulation_header import *
from point_cloud_filtering.srv import DetectHandle
from orion_actions.msg import *


class OpenDoorAction(object):


    def __init__(self, name):
        self._action_name = 'open_door'
        self._as = actionlib.SimpleActionServer(self._action_name, 	orion_actions.msg.OpenDoorAction,execute_cb=self.execute_cb, auto_start=False)
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

    def get_handle_pose(self):
        # This service has a timeout of 10s
        rospy.wait_for_service('/handle_detection')
        try:
            detect_handle_service = rospy.ServiceProxy('/handle_detection', DetectHandle)
            response = detect_handle_service(True)
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e

        return response

    def pull_down_handle(self):
        self.whole_body.move_end_effector_pose(geometry.pose(y=-0.06), 'hand_palm_link')
        return True

    def execute_cb(self, goal_msg):
        rospy.loginfo('%s: Executing callback. Moving into position' % (self._action_name))
        self.whole_body.move_to_go()
        self.gripper.command(1)

        self.whole_body.collision_world = None
        self.whole_body.linear_weight = 50.0
        rospy.loginfo('%s: Calling handle detection...' % (self._action_name))
        rospy.sleep(1)
        self.tts.say("I'm now looking for the door handle")
        handle_pose = self.get_handle_pose()

        if handle_pose.handle_detected == False:
            rospy.loginfo('%s: Could not find door handle. Handle detection timed out.' % self._action_name)
            self._as.set_aborted()

        if self._as.is_preempt_requested():
            rospy.loginfo('%s: Preempted. Moving to go and exiting.' % self._action_name)
            self.whole_body.move_to_go()
            self._as.set_preempted()
            return

        rospy.loginfo('%s: Grasping handle...' % (self._action_name))
        self.tts.say("Door handle found. Moving to grasp.")
        rospy.sleep(1)
        self.whole_body.move_end_effector_pose(geometry.pose(x=handle_pose.x, y=handle_pose.y+0.03,  z=handle_pose.z-0.8), 'head_rgbd_sensor_rgb_frame')

        # Determine if door hinge is on left or right
        if handle_pose.x > 0:
            hinge_sign = 1 # hinge on left
        else:
            hinge_sign = -1 # hinge on right

        try:
            self.whole_body.move_end_effector_pose(geometry.pose(z=0.02), 'hand_palm_link')
        except:
            rospy.loginfo("%s: Couldn't move forward..." % (self._action_name))
            pass

        self.gripper.apply_force(self._GRASP_FORCE)
        rospy.sleep(2)

        rospy.loginfo('%s: Executing opening motion...' % (self._action_name))
        self.tts.say("Grasped successfully. Now opening.")
        rospy.sleep(1)

        try:
            self.whole_body.move_end_effector_pose(geometry.pose(y=0.025), 'hand_palm_link')
            rospy.loginfo('%s: Successfully pulled handle down...' % (self._action_name))
        except:
            rospy.loginfo('%s: Failed to move to the side...' % (self._action_name))

        rospy.sleep(1)

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
        except:
            self.tts.say("Sorry. I encountered a problem.")
            rospy.sleep(1)
            self.gripper.set_distance(0.1)
            self.whole_body.move_end_effector_pose(geometry.pose(z=-0.1), 'hand_palm_link')

        self.tts.say("I will now try to open the door fully.")
        rospy.sleep(1)
        self.whole_body.move_to_go()
        self.whole_body.move_to_neutral()
        self.whole_body.linear_weight = 100
        self.omni_base.go_rel(0, -hinge_sign*0.15, 0)
        self.whole_body.move_to_joint_positions({'arm_lift_joint': 0.5})
        self.whole_body.move_end_effector_pose(geometry.pose(z=0.3), 'hand_palm_link')
        self.omni_base.go_rel(0.15, 0, 0)
        self.omni_base.go_rel(0, 0, hinge_sign * math.pi/2)
        self.whole_body.move_to_go()
        rospy.loginfo('%s: Succeeded door opening. Now returning results.' % self._action_name)
        self.tts.say("Door opening complete.")
        # self.tts.say("Yeah boy.")
        rospy.sleep(1)

        result = OpenDoorResult()
        result.result = True
        self.whole_body.planning_timeout = 20.0
        self._as.set_succeeded(result)


if __name__ == '__main__':
    rospy.init_node('open_door_server_node')
    server = OpenDoorAction(rospy.get_name())
    rospy.spin()
