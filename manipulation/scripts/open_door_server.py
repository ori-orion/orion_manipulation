#! /usr/bin/env python

import hsrb_interface
import hsrb_interface.geometry as geometry
import numpy as np
import rospy
import actionlib
import json
import rospkg
import tf

from manipulation.manipulation_header import *
from tmc_manipulation_msgs.msg import CollisionObject
from manipulation.msg import *

class OpenDoorAction(object):


    def __init__(self, name):
        self._action_name = 'open_door'       
	self._as = actionlib.SimpleActionServer(self._action_name, 	manipulation.msg.OpenDoorAction,execute_cb=self.execute_cb, auto_start=False)
        self._as.start()
	
        # Preparation for using the robot functions
        self.robot = hsrb_interface.Robot()
        self.whole_body = self.robot.try_get('whole_body')
        self.omni_base = self.robot.try_get('omni_base')
        self.collision_world = self.robot.try_get('global_collision_world')
        self.gripper = self.robot.try_get('gripper')
        self._HAND_TF = 'hand_palm_link'
        
	self._GRASP_FORCE = 0.8

	self.callback_counter = 0

	# Increase planning timeout. Default is 10s
	self.whole_body.planning_timeout = 20.0

	# Set up publisher for the collision map
	self.br = tf.TransformBroadcaster()

	self.pub = rospy.Publisher('known_object', CollisionObject, queue_size=1)
	rospy.loginfo('%s: Initialised. Ready for clients.' % ( self._action_name))

    def callback(self, msg):
        # Get the message
	
	if self.callback_counter == 0:
		self.callback_counter +=1			
		message = msg


		# Get only the stuff in front
		x_list = []
		y_list = []
		for i in range(len(message.poses)):
		    pose = message.poses[i]
		    x_list.append(pose.position.x)
		    y_list.append(pose.position.y)

		median_x = np.median(x_list)
		median_y = np.median(y_list)


		inds_to_remove = []
		for i in range(len(message.poses)):
		    pose = message.poses[i]
		    if ( (pose.position.x >= median_x + 0.2) or (pose.position.x <= median_x - 0.2) or
		         (pose.position.y >= median_y + 0.2) or (pose.position.y <= median_y - 0.2) or 
			 (pose.position.z >= 1.2)):
		        inds_to_remove.append(i)


		for index in sorted(inds_to_remove, reverse=True):
		    del message.poses[index], message.shapes[index]


		# Now try to find handle
		# Find which boxes to removes
		inds_to_remove = []
		x_list = []
		y_list = []
		z_list = []
		
		
		rospy.loginfo('%s: Finding modal values.' % ( self._action_name))
		for i in range(len(message.poses)):
		    pose = message.poses[i]
		    x_list.append(pose.position.x)
		    y_list.append(pose.position.y)
 		    z_list.append(pose.position.z)

		mode_x = max(set(x_list), key=x_list.count)
		mode_y = max(set(y_list), key=y_list.count)
		mode_z = max(set(z_list), key=z_list.count)
		comp_list = [mode_x, mode_y, mode_z]
	
		max_x = -1000
		max_y = -1000
		max_z = -1000
		min_x = 1000
		min_y = 1000
		min_z = 1000

		inds_to_remove = []
		
		rospy.loginfo('%s: Finding indices to remove.' % ( self._action_name))
		if max(comp_list) == mode_x:
			for i in range(len(message.poses)):
			    pose = message.poses[i]
			    if pose.position.x == mode_x:
		                max_y = max([pose.position.y, max_y])
				max_z = max([pose.position.z, max_z])
				min_y = min([pose.position.y, min_y])
				min_z = min([pose.position.z, min_z])
			for i in range(len(message.poses)):
			    pose = message.poses[i]
			    if (pose.position.y < min_y or pose.position.y > max_y or
				 pose.position.z < min_z or pose.position.z > max_z):
			        inds_to_remove.append(i)

		if max(comp_list) == mode_y:
			for i in range(len(message.poses)):
			    pose = message.poses[i]
			    if pose.position.y == mode_y:
				max_x = max([pose.position.x, max_x])
				max_z = max([pose.position.z, max_z])
				min_x = min([pose.position.x, min_x])
				min_z = min([pose.position.z, min_z])
			for i in range(len(message.poses)):
			    pose = message.poses[i]
			    if (pose.position.x < min_x or pose.position.x > max_x or
				 pose.position.z < min_z or pose.position.z > max_z):
			        inds_to_remove.append(i)

		if max(comp_list) == mode_z:
			for i in range(len(message.poses)):
			    pose = message.poses[i]
			    if pose.position.z == mode_z: 
				max_y = max([pose.position.y, max_y])
				max_x = max([pose.position.x, max_x])
				min_y = min([pose.position.y, min_y])
				min_x = min([pose.position.x, min_x])
			for i in range(len(message.poses)):
			    pose = message.poses[i]
			    if (pose.position.y < min_y or pose.position.y > max_y or
				 pose.position.x < min_x or pose.position.x > max_x):
			        inds_to_remove.append(i)

		rospy.loginfo('%s: Removing indices.' % ( self._action_name))

		for index in sorted(inds_to_remove, reverse=True):
		    del message.poses[index], message.shapes[index]

		rospy.loginfo('%s: Calculating new tf.' % ( self._action_name))
		
		# Should now be left with the handle so filter again for the 

		#tf_listener = tf.TransformListener()
		#    foundTrans = False
		#    while not foundTrans:
		#	try:
		#	    t = tf_listener.getLatestCommonTime("/map", 'hand_palm_link')
		#	    (trans, rot) = tf_listener.lookupTransform('/map', object_tf, rospy.Time(0))
		#	    foundTrans = True
		#	except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
		#	    continue

    		#hand_x = trans[0]
    		#hand_y = trans[1]
		#hand_x_diff = 1000
		#hand_y_diff = 1000

		#for i in range(len(message.poses)):
		#    pose = message.poses[i]
		#   hand_x_diff = min(hand_x_diff, abs(hand_x-pose.position.x))
		#    hand_y_diff = min(hand_y_diff, abs(hand_y-pose.position.y))

		#inds_to_remove = []

		#if min(hand_x_diff,hand_y_diff) == hand_x_diff:
		#	for i in range(len(message.poses)):
		#	    pose = message.poses[i]
		#	    if ( (pose.position.x >= median_x + 0.2) or (pose.position.x <= median_x - 0.2) or
		#		 (pose.position.y >= median_y + 0.2) or (pose.position.y <= median_y - 0.2) or 
		#		 (pose.position.z >= 1.5)):
			        
    		#	        inds_to_remove.append(i)


		#if min(hand_x_diff,hand_y_diff) == hand_y_diff:


		
	








		coord1 = []
		coord2 = []
		coord3 = []
		coord4 = []
		coord5 = []
		coord6 = []
		coord7 = []

		num_points = len(message.poses) 

		for i in range(num_points):
		    pose = message.poses[i]
		    coord1.append(pose.position.x)
		    coord2.append(pose.position.y)
 		    coord3.append(pose.position.z)
		    coord4.append(pose.orientation.x)
		    coord5.append(pose.orientation.y)
		    coord6.append(pose.orientation.z)
		    coord7.append(pose.orientation.w)

		# Find the average TF frame
		coord1 = sum(coord1) / float(num_points)
		coord2 = sum(coord2) / float(num_points)
 		coord3 = sum(coord3) / float(num_points)
		coord4 = sum(coord4) / float(num_points)
		coord5 = sum(coord5) / float(num_points)
		coord6 = sum(coord6) / float(num_points)
		coord7 = sum(coord7) / float(num_points)

		reset_collision_map_build()

		# Publish the filtered message
		self.pub.publish(message)
		
		# Publish the filtered message
		rospy.loginfo('%s: Sending transform.' % ( self._action_name))
		self.br.sendTransform((coord1, coord2, coord3),
                        (coord4,coord5,coord6,coord7),
                        rospy.Time.now(),
                        "handle",
                        "map")


    

    def execute_cb(self, goal_msg):
	 
	get_collision_map(self.robot)

	# Publish the tf of the destination
	rospy.loginfo('%s: Suscribing to the collision environment and activating callback.' % ( self._action_name))
        rospy.Subscriber("known_object", CollisionObject, self.callback)
       
	_result = OpenDoorResult()
        rospy.loginfo('%s: Succeeded' % self._action_name)
        _result.goal_complete = True
	self._as.set_succeeded(_result)
	
	# Reset callback counter
	self.callback_counter = 0


if __name__ == '__main__':
    rospy.init_node('open_door_server_node')
    server = OpenDoorAction(rospy.get_name())
    rospy.spin()
