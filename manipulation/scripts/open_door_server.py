#! /usr/bin/env python

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


from manipulation.manipulation_header import *
from manipulation.msg import *
from point_cloud_filtering.srv import DetectHandle

def get_handle_pose():
    rospy.wait_for_service('/handle_detection')
    try:
        detect_handle_service = rospy.ServiceProxy('/handle_detection', DetectHandle)
        response = detect_handle_service(True)
        #print "tmc_reconstruction started."
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e

    return response

class OpenDoorAction(object):


    def __init__(self, name):
        self._action_name = 'open_door'       
	self._as = actionlib.SimpleActionServer(self._action_name, 	manipulation.msg.OpenDoorAction,execute_cb=self.execute_cb, auto_start=False)
        self._as.start()
	
        # Preparation for using the robot functions
        self.robot = hsrb_interface.Robot()
        self.whole_body = self.robot.try_get('whole_body')
        self.gripper = self.robot.try_get('gripper')
        self._HAND_TF = 'hand_palm_link'
        
	self._GRASP_FORCE = 0.8


	# Increase planning timeout. Default is 10s
	self.whole_body.planning_timeout = 20.0

	rospy.loginfo('%s: Initialised. Ready for clients.' % ( self._action_name))
    def execute_cb(self, goal_msg):
	 
	self.whole_body.move_to_go()
	self.gripper.command(0.1)

	handle_pose = get_handle_pose()

	self.whole_body.move_end_effector_pose(geometry.pose(x=handle_pose.x, y=handle_pose.y,  z=handle_pose.z-0.06), 'head_rgbd_sensor_rgb_frame')

	self.gripper.apply_force(self._GRASP_FORCE)

	_result = OpenDoorResult()
        rospy.loginfo('%s: Succeeded' % self._action_name)
        _result.goal_complete = True
	self._as.set_succeeded(_result)


if __name__ == '__main__':
    rospy.init_node('open_door_server_node')
    server = OpenDoorAction(rospy.get_name())
    rospy.spin()
