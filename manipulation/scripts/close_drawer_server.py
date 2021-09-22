#! /usr/bin/env python3
""" Action server for closing a drawer.
This assumes that the handle is already grasped and uses a hard-coded forward
motion to close the drawer. Includes speech commentary
"""
__author__ = "Mark Finean"
__email__ = "mfinean@robots.ox.ac.uk"

import time
import hsrb_interface
import hsrb_interface.geometry as geometry
import numpy as np
import rospy
import actionlib

import math
from hsrb_interface import robot as _robot

_robot.enable_interactive()

from manipulation.manipulation_header import *
from orion_actions.msg import *


class CloseDrawerAction(object):


    def __init__(self, name):
        self._action_name = 'close_drawer'
        self._as = actionlib.SimpleActionServer(self._action_name, 	orion_actions.msg.CloseDrawerAction,execute_cb=self.execute_cb, auto_start=False)
        self._as.start()

        # Preparation for using the robot functions
        self.robot = hsrb_interface.Robot()
        self.whole_body = self.robot.try_get('whole_body')
        self.omni_base = self.robot.try_get('omni_base')
        self.gripper = self.robot.try_get('gripper')
        self._HAND_TF = 'hand_palm_link'
        self._GRASP_FORCE = 0.4
        self.tts = self.robot.try_get('default_tts')
        self.tts.language = self.tts.ENGLISH

        # Increase planning timeout. Default is 10s
        self.whole_body.planning_timeout = 20.0

        rospy.loginfo('%s: Initialised. Ready for clients.' % (self._action_name))


    def execute_cb(self, goal_msg):
        rospy.loginfo('%s: Executing callback. Closing the drawer.' % (self._action_name))
        self.tts.say("I will now closer the drawer.")
        rospy.sleep(1)

        # Push forward 25cm
        self.whole_body.move_end_effector_pose(geometry.pose(z=0.25), 'hand_palm_link')
        rospy.loginfo('%s: Drawer closed.' % (self._action_name))

        # Let go of handle and return to go
        self.tts.say("Letting go of the handle.")
        rospy.sleep(1)
        self.gripper.command(1)
        self.whole_body.move_end_effector_pose(geometry.pose(z=-0.1), 'hand_palm_link')
        self.whole_body.move_to_go()

        rospy.loginfo('%s: Succeeded closing drawer.' % self._action_name)
        self.tts.say("Completed drawer closing.")
        rospy.sleep(1)

        result = CloseDrawerResult()
        result.result = True
        self._as.set_succeeded(result)


if __name__ == '__main__':
    rospy.init_node('close_drawer_server_node')
    server = CloseDrawerAction(rospy.get_name())
    rospy.spin()
