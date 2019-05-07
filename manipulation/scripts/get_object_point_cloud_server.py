#! /usr/bin/env python

import hsrb_interface
import hsrb_interface.geometry as geometry
import numpy as np
import rospy
import actionlib

from actionlib_msgs.msg import GoalStatus

from manipulation.manipulation_header import *
from tmc_manipulation_msgs.msg import CollisionObject
from manipulation.msg import *


class FilterPointCloudAction(object): 

    def __init__(self, name):
        self._action_name = 'get_object_point_cloud'
        self._as = actionlib.SimpleActionServer(self._action_name, manipulation.msg.FilterPointCloudAction,execute_cb=self.execute_cb, auto_start=False)
        self._as.start()

	self.exclusion_bounds = np.array([0.08, 0.08, 0.08])
 	self.object_tf = ''
	self.upper_bounds = []
	self.lower_bounds = []


    def callback(self, msg):
        # Get the message
        message = msg

        # Find which boxes to removes
        inds_to_remove = []
        for i in range(len(message.poses)):
            pose = message.poses[i]
            if ((pose.position.x >= self.upper_bounds[0] or pose.position.x <= self.lower_bounds[0] or
                 pose.position.y >= self.upper_bounds[1] or pose.position.y <= self.lower_bounds[1] or
                 pose.position.z >= self.upper_bounds[2] or pose.position.z <= self.lower_bounds[2])):

                inds_to_remove.append(i)

        # Remove the boxes
        for index in sorted(inds_to_remove, reverse=True):
            del message.poses[index], message.shapes[index]

        # Publish the filtered message
        self.pub.publish(message)

    def execute_cb(self, goal_msg):

	object_tf = goal_msg.goal_tf
	rospy.loginfo('%s: Filtering point cloud for :' + str(object_tf) % (self._action_name))
	goal_object_pose = get_object_pose(goal_tf)
	
	# Filters out the objects pose with an exclusion zone around it.
	self.upper_bounds = goal_object_pose + self.exclusion_bounds 
        self.lower_bounds = goal_object_pose - self.exclusion_bounds

	rospy.Subscriber("known_object_pre_filter", CollisionObject, self.callback)

	_result = FilterPointCloudResult()
	_result.goal_complete = True
        self._as.set_succeeded(_result)


if __name__ == '__main__':
    rospy.init_node('get_object_point_cloud_server_node')
    server = FilterPointCloudAction(rospy.get_name())
    rospy.spin()
