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
from manipulation.msg import *

class PickUpObjectAction(object):
    # create messages that are used to publish feedback/result
    _feedback = PickUpObjectActionFeedback()
    _result = PickUpObjectActionResult()

    def __init__(self, name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, 	manipulation.msg.PickUpObjectAction,execute_cb=self.execute_cb, auto_start=False)
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

	# Load the config file
	rospack = rospkg.RosPack()
	pkg_path = rospack.get_path('manipulation')
	config_path = pkg_path + '/src/manipulation/config.json'
	with open(config_path) as f:
		self.config = json.load(f)
		f.close()

	# Remove any collsion markers further than this specification away from the object	
	self.excess_bounds = np.array([2.0, 2.0, 2.0])

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
        pub.publish(message)


    def execute_cb(self, goal_msg):

        success = True
        goal_tf = goal_msg.goal_tf
	self.whole_body.end_effector_frame = 'hand_palm_link'

	# Get the grasp type from the config file
	try:
		grasp_type = self.config[goal_tf]['grasp_pose']
	except:
		grasp_type = 'horizontal'
	
	# Put default poses and bounds about the tf frame of object to remove from map
	try:
		exclusion_bounds = self.config[goal_tf]['estimated_halfsize']
	except:
		exclusion_bounds = np.array([0.07, 0.07, 0.07])

	# Put default poses and bounds about the tf frame of object to remove from map
	try:
		grasp_offset = self.config[goal_tf]['offset']
	except:
		grasp_offset = 0

	grasp_pose_dict = {'horizontal': [geometry.pose(x=-0.05, z=0.05, ek=-1.57), geometry.pose(z=0.03)], 
				'horizontal_rotate':[geometry.pose(x=-0.07, z=0.02, ei=1.57) , geometry.pose(z=0.02)], 
				'above': [geometry.pose(z=0.10, ei=3.14), geometry.pose(z=0.05)], 
				'above_offset':[geometry.pose(y=grasp_offset, z=-0.05, ek=-1.57), geometry.pose(z=0.03)], 
				'suction':[geometry.pose(z=0.05, ei=3.14), geometry.pose(z=0.045)], 
				'ar':[geometry.pose(z=-0.05, ek=-1.57), geometry.pose(z=0.03)]}
	
	rospy.loginfo('%s: Chosen grasp type - "%s" of type "%s".' % ( self._action_name,grasp_type, type(grasp_type)))

	# Get the appropriate grasp and pre-grasp poses depending on the type of object
	chosen_pregrasp_pose = grasp_pose_dict[grasp_type][0]
	chosen_grasp_pose = grasp_pose_dict[grasp_type][1]

	# Get the appropriate grasp and pre-grasp poses depending on the type of object
	#if grasp_type == 'horizontal':
	#	chosen_pregrasp_pose = geometry.pose(x=-0.05, z=0.05, ek=-1.57)
	#	chosen_grasp_pose = geometry.pose(z=0.03)
	#elif grasp_type == 'horizontal_rotate':
	#	chosen_pregrasp_pose = geometry.pose(x=-0.07, z=0.02, ei=1.57)
	#	chosen_grasp_pose = geometry.pose(z=0.02)
	#	exclusion_bounds = np.array([0.07, 0.07, 0.02])
	#elif grasp_type == 'above':
	#	chosen_pregrasp_pose = geometry.pose(z=0.10, ei=3.14)
	#	chosen_grasp_pose = geometry.pose(z=0.05)
	#elif grasp_type == 'above_offset':
	#	grasp_offset = self.config[goal_tf]['offset']
	#	chosen_pregrasp_pose = geometry.pose(y=grasp_offset, z=-0.05, ek=-1.57)
	#	chosen_grasp_pose = geometry.pose(z=0.03)
	#elif grasp_type == 'suction':
	#	chosen_pregrasp_pose = geometry.pose(z=0.05, ei=3.14)
	#	chosen_grasp_pose = geometry.pose(z=0.045)
	#	self.whole_body.end_effector_frame = 'hand_l_finger_vacuum_frame'
	#elif grasp_type == 'ar':
	#	chosen_pregrasp_pose = geometry.pose(z=-0.05, ek=-1.57)
	#	chosen_grasp_pose = geometry.pose(z=0.03)
	
	
        # publish info to the console for the user
        rospy.loginfo('%s: Planning to reach %s using grasp type "%s".' % ( self._action_name, goal_tf,grasp_type))

        global pub, lower_bounds, upper_bounds, excess_lower_bounds, excess_upper_bounds

        # ---------------------------------- Now begin the actual actions --------------------
        #rospy.loginfo('%s: Getting in move_to_go position.' % ( self._action_name))
	#self.whole_body.move_to_go()
	
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
        rospy.loginfo('%s: Collision Map generated.' % (self._action_name))

        # Get the object pose to subtract from collision map
        rospy.loginfo('%s: Getting object pose.' % (self._action_name))
        goal_object_pose = get_object_pose(goal_tf)
        upper_bounds = goal_object_pose + exclusion_bounds
        lower_bounds = goal_object_pose - exclusion_bounds
	
 	excess_lower_bounds = goal_object_pose - self.excess_bounds 
	excess_upper_bounds = goal_object_pose + self.excess_bounds 

        rospy.loginfo('%s: Bounds have been set' % self._action_name)

        # Remove object from map and publish to correct topic
        rospy.loginfo('%s: Modifying collision map.' % (self._action_name))
        pub = rospy.Publisher('known_object', CollisionObject, queue_size=1)
        rospy.Subscriber("known_object_pre_filter", CollisionObject, self.callback)
        rospy.sleep(2)
       
        # Set up listener to find the object
        rospy.loginfo('%s: Finding object.' % (self._action_name))
	check_for_object(goal_tf)

        # Turn on collision checking
        self.whole_body.collision_world = self.collision_world
	rospy.sleep(2)

	# Move to pregrasp
        rospy.loginfo('%s: Moving to pre-grasp position.' % (self._action_name))
	self.whole_body.move_end_effector_pose(chosen_pregrasp_pose, goal_tf)

	# Turn off collision checking to get close and grasp
	try: 
		rospy.loginfo('%s: Turning off collision checking to get closer.' % (self._action_name))
		self.whole_body.collision_world = None
		rospy.loginfo('%s: Moving to grasp position.' % (self._action_name))	
		self.whole_body.move_end_effector_pose(chosen_grasp_pose, self.whole_body.end_effector_frame)
	except:
		rospy.loginfo('%s: Error in trying to grasp. Returning to neutral pose.' % (self._action_name))
		self.whole_body.move_to_neutral()	
	
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
			    rospy.Duration(_CONNECTION_TIMEOUT)):
		        raise Exception(suction_action + ' does not exist')
	        except Exception as e:
		    rospy.logerr(e)
		    sys.exit(1)

	        # Send a goal to start suction
	        rospy.loginfo('%s: Suction server found. Activating suction...' % (self._action_name))
	        suction_on_goal = SuctionControlGoal()
	        suction_on_goal.timeout = _SUCTION_TIMEOUT
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
	rospy.loginfo('%s: Try to move to go.' % self._action_name)
	self.whole_body.move_to_go()

        if success:
            # self._result.sequence = self._feedback.sequence
            rospy.loginfo('%s: Succeeded' % self._action_name)
            # self._as.set_succeeded(self._result)
            self._as.set_succeeded(1)


if __name__ == '__main__':
    rospy.init_node('pick_up_object_server_node')
    server = PickUpObjectAction(rospy.get_name())
    rospy.spin()
