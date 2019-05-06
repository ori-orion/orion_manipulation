#! /usr/bin/env python

import hsrb_interface
import hsrb_interface.geometry as geometry
import numpy as np
import rospy
import actionlib
import json
import rospkg

from actionlib_msgs.msg import GoalStatus
from tmc_suction.msg import (
    SuctionControlAction,
    SuctionControlGoal
)
from manipulation.manipulation_header import *
from tmc_manipulation_msgs.msg import CollisionObject
from orion_actions.msg import *

class PickUpObjectAction(object):

    def __init__(self, name):
        self._action_name = 'pick_up_object'
        #self._action_name = name
	print("Action name is: " + name)        
	self._as = actionlib.SimpleActionServer(self._action_name, 	orion_actions.msg.PickUpObjectAction,execute_cb=self.execute_cb, auto_start=False)
        self._as.start()
	
        # Preparation for using the robot functions
        self.robot = hsrb_interface.Robot()
        self.whole_body = self.robot.try_get('whole_body')
        self.omni_base = self.robot.try_get('omni_base')
        self.collision_world = self.robot.try_get('global_collision_world')
        self.gripper = self.robot.try_get('gripper')
        self._HAND_TF = 'hand_palm_link'
        self._GRASP_FORCE = 0.8
	self.whole_body.end_effector_frame = 'hand_palm_link'

	# Define the vacuum timeouts
	self._CONNECTION_TIMEOUT = 15.0
	self._SUCTION_TIMEOUT = rospy.Duration(20.0)

	# Increase planning timeout. Default is 10s
	self.whole_body.planning_timeout = 20.0

	# Set up publisher for the collision map
	self.pub = rospy.Publisher('known_object', CollisionObject, queue_size=1)

	rospy.loginfo('%s: Initialised. Ready for clients.' % ( self._action_name))

    def callback(self, msg):
        # Get the message		
	message = msg

	rospy.loginfo('%s: Removing excess collision space.' % ( self._action_name))
	
	# Find which boxes to removes
	inds_to_remove = []
	for i in range(len(message.poses)):
	    pose = message.poses[i]
	    if ((pose.position.x <= upper_bounds[0] and pose.position.x >= lower_bounds[0] and
	            pose.position.y <= upper_bounds[1] and pose.position.y >= lower_bounds[1] and
	            pose.position.z <= upper_bounds[2] and pose.position.z >= lower_bounds[2]) or 
		(pose.position.x <= excess_lower_bounds[0] or pose.position.x >= excess_upper_bounds[0] or
	            pose.position.y <= excess_lower_bounds[1] or pose.position.y >= excess_upper_bounds[1] or
	            pose.position.z <= excess_lower_bounds[2] or pose.position.z >= excess_upper_bounds[2])):
	        inds_to_remove.append(i)

	# Remove the boxes
	for index in sorted(inds_to_remove, reverse=True):
	    del message.poses[index], message.shapes[index]

	# Publish the filtered message
	self.pub.publish(message)


    def execute_cb(self, goal_msg):
        success = False

        goal_tf = goal_msg.goal_tf
	goal_tf = get_similar_tf(goal_tf)
        rospy.loginfo('{0}: Choosing tf frame "{1}".'.format(self._action_name, str(goal_tf)))

	if goal_tf is None:
		self._as.set_aborted()
                rospy.loginfo('{0}: Found no similar tf frame. Aborting.'.format(self._action_name))
		return

	# ------------------------------------------------------------------------------
	# Check the object is in sight
	rospy.loginfo('%s: Checking object is in sight...' % ( self._action_name))
	check_for_object(goal_tf)

	# Look at the object - this is to make sure that we get all of the necessary collision map
	rospy.loginfo('%s: Moving head to look at the object.' % ( self._action_name))
	self.whole_body.gaze_point(ref_frame_id=goal_tf)
        
	# Open or close gripper
	if grasp_type == 'suction':
		rospy.loginfo('%s: Closing gripper.' % (self._action_name))
		self.gripper.command(0.1)
        else:
		rospy.loginfo('%s: Opening gripper.' % (self._action_name))
		self.gripper.command(1.2)

        # Follow the gripper
        self.whole_body.looking_hand_constraint = True

        # Set collision map
        rospy.loginfo('%s: Getting Collision Map.' % (self._action_name))
        get_collision_map(self.robot)
        rospy.sleep(2)
	rospy.loginfo('%s: Collision Map generated.' % (self._action_name))
		
        # Get the object pose to subtract from collision map
	rospy.loginfo('%s: Checking object is still in sight...' % ( self._action_name))
	check_for_object(goal_tf)
	rospy.loginfo('%s: Getting object pose.' % (self._action_name))
        goal_object_pose = get_object_pose(goal_tf)
	rospy.sleep(1)
        
	# Alter collision map
	upper_bounds = goal_object_pose + exclusion_bounds
        lower_bounds = goal_object_pose - exclusion_bounds
 	excess_lower_bounds = goal_object_pose - self.excess_bounds
	excess_upper_bounds = goal_object_pose + self.excess_bounds 
        rospy.loginfo('%s: Bounds have been set' % self._action_name)

        # Remove object from map and publish to correct topic
        rospy.loginfo('%s: Modifying collision map.' % (self._action_name))
        rospy.Subscriber("known_object_pre_filter", CollisionObject, self.callback)
       
        # Set up listener to find the object
        rospy.loginfo('%s: Finding object.' % (self._action_name))
	check_for_object(goal_tf)

        # Turn on collision checking
	rospy.sleep(1)
        self.whole_body.collision_world = self.collision_world
	rospy.sleep(2)
	
	# Give opportunity to preempt
	if self._as.is_preempt_requested():
        	rospy.loginfo('%s: Preempted' % self._action_name)
           	self._as.set_preempted()
           	success = False
	  	# Reset callback counter
		self.callback_counter = 0
           	return

	try: 
		# Move to pregrasp
        	rospy.loginfo('%s: Moving to pre-grasp position.' % (self._action_name))
		self.whole_body.move_end_effector_pose(chosen_pregrasp_pose, goal_tf)

		# Turn off collision checking to get close and grasp
		rospy.loginfo('%s: Turning off collision checking to get closer.' % (self._action_name))
		self.whole_body.collision_world = None
		rospy.loginfo('%s: Moving to grasp position.' % (self._action_name))	
		self.whole_body.move_end_effector_pose(chosen_grasp_pose, self.whole_body.end_effector_frame)
	except Exception as e:
		rospy.loginfo('{0}: Encountered exception {1}.'.format(self._action_name, str(e)))
		self.whole_body.collision_world = None
		rospy.loginfo('%s: Returning to neutral pose.' % (self._action_name))
		#self.omni_base.go_rel(-0.3,0,0)
		self.whole_body.move_to_neutral()
		self._as.set_aborted()
		success = False
		# Reset callback counter
		self.callback_counter = 0
           	return
	
	# Use suction or gripper to grab the object
	if grasp_type == 'suction':
		rospy.loginfo('%s: Turning on on the suction...' % (self._action_name))

	        # Create action client to control suction
	        suction_action = '/hsrb/suction_control'
	        suction_control_client = actionlib.SimpleActionClient(
		    suction_action, SuctionControlAction)

	        # Wait for connection
	        try:
		    if not suction_control_client.wait_for_server(
			    rospy.Duration(self._CONNECTION_TIMEOUT)):
		        raise Exception(suction_action + ' does not exist')
	        except Exception as e:
		    rospy.logerr(e)
		    #sys.exit(1)

	        # Send a goal to start suction
	        rospy.loginfo('%s: Suction server found. Activating suction...' % (self._action_name))
	        suction_on_goal = SuctionControlGoal()
	        suction_on_goal.timeout = self._SUCTION_TIMEOUT
	        suction_on_goal.suction_on.data = True
	        
		if (suction_control_client.send_goal_and_wait(suction_on_goal) ==
		        GoalStatus.SUCCEEDED):
		    rospy.loginfo('Suction succeeded. Object picked up')
	        else:
		    rospy.loginfo('Suction failed')

	else:
		# Specify the force to grasp
		self.gripper.apply_force(self._GRASP_FORCE)
		rospy.sleep(0.5)
		rospy.loginfo('%s: Object grasped.' % self._action_name)

	# Turn collision checking back on
	self.whole_body.collision_world = self.collision_world
	rospy.sleep(0.5)

	# Now return to moving position
	try:
		rospy.loginfo('%s: Try to move to go.' % self._action_name)
		self.omni_base.go_rel(-0.3,0,0)
		self.whole_body.move_to_go()
	except Exception as e:
		rospy.loginfo('{0}: Encountered exception {1}.'.format(self._action_name, str(e)))
                try:
			rospy.loginfo('%s: Moving back and attempting to move to go again without collision detection.' % self._action_name)
			try:
				self.omni_base.go_rel(-0.3,0,0)
			except:
				try:
					self.omni_base.go_rel(0,0.3,0)	
				except:
					self.omni_base.go_rel(0,-0.3,0)	
			self.whole_body.move_to_go()
		except:
			self.whole_body.collision_world = None
			self.omni_base.go_rel(-0.3,0,0)
			self.whole_body.move_to_go()
			
	_result = PickUpObjectResult()
        rospy.loginfo('%s: Succeeded' % self._action_name)
        _result.goal_complete = True
	self._as.set_succeeded(_result)
	
	# Reset callback counter
	self.callback_counter = 0


if __name__ == '__main__':
    rospy.init_node('pick_up_object_server_node')
    server = PickUpObjectAction(rospy.get_name())
    rospy.spin()
