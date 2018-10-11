#! /usr/bin/env python

import hsrb_interface
import hsrb_interface.geometry as geometry
import numpy as np
import rospy
import actionlib

from actionlib_msgs.msg import GoalStatus
from tmc_suction.msg import (
    SuctionControlAction,
    SuctionControlGoal
)

class GiveObjectToOperatorAction(object):
    # create messages that are used to publish feedback/result
    _feedback = GiveObjectToOperatorActionFeedback()
    _result = GiveObjectToOperatorActionResult()

    def __init__(self, name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, manipulation.msg.GiveObjectToOperatorAction,execute_cb=self.execute_cb, auto_start=False)
        self._as.start()

        # Preparation for using the robot functions
        self.robot = hsrb_interface.Robot()
        self.whole_body = self.robot.try_get('whole_body')
        self.omni_base = self.robot.try_get('omni_base')
        self.collision_world = self.robot.try_get('global_collision_world')
        self.gripper = self.robot.try_get('gripper')

        self._HAND_TF = 'hand_palm_link'
        self._GRASP_FORCE = 0.2

	# Define the vacuum timeouts
	self._CONNECTION_TIMEOUT = 15.0
	self._SUCTION_TIMEOUT = rospy.Duration(20.0)

	# Increase planning timeout. Default is 10s
	self.whole_body.planning_timeout = 20.0


    def execute_cb(self, goal_msg):

        success = True
        
	rospy.loginfo('%s: Opening gripper.' % (self._action_name))
	self.gripper.command(1.2)


        # Create action client to control suction
        suction_action = '/hsrb/suction_control'
        suction_control_client = actionlib.SimpleActionClient(
	    suction_action, SuctionControlAction)

        # Wait for connection
	rospy.loginfo('%s: Connecting to suction server...' % (self._action_name))
        try:
	    if not suction_control_client.wait_for_server(
		    rospy.Duration(_CONNECTION_TIMEOUT)):
	        raise Exception(suction_action + ' does not exist')
        except Exception as e:
	    rospy.logerr(e)
	    sys.exit(1)

	rospy.loginfo('%s: Turning off the suction...' % (self._action_name))

	# Send a goal to stop suction
	suction_off_goal = SuctionControlGoal()
	suction_off_goal.suction_on.data = False
	suction_control_client.send_goal_and_wait(suction_off_goal)

        if success:
            # self._result.sequence = self._feedback.sequence
            rospy.loginfo('%s: Succeeded' % self._action_name)
            # self._as.set_succeeded(self._result)
            self._as.set_succeeded(1)


if __name__ == '__main__':
    rospy.init_node('give_object_to_operator_server_node')
    server = GiveObjectToOperatorAction(rospy.get_name())
    rospy.spin()