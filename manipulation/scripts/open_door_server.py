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
import math

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


        # Increase planning timeout. Default is 10s
        self.whole_body.planning_timeout = 20.0

        rospy.loginfo('%s: Initialised. Ready for clients.' % (self._action_name))

    def get_handle_pose(self):
        rospy.wait_for_service('/handle_detection')
        try:
            detect_handle_service = rospy.ServiceProxy('/handle_detection', DetectHandle)
            response = detect_handle_service(True)
            #print "tmc_reconstruction started."
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e

        return response

    def pull_down_handle(self):
        self.whole_body.move_end_effector_pose(geometry.pose(y=-0.04), 'hand_palm_link')
        return True

    def execute_cb(self, goal_msg):
        rospy.loginfo('%s: Executing callback. Moving into position' % (self._action_name))
        self.whole_body.move_to_go()
        self.gripper.command(1)

        self.whole_body.collision_world = None
        self.whole_body.linear_weight = 50.0
        rospy.loginfo('%s: Calling handle detection...' % (self._action_name))
        handle_pose = self.get_handle_pose()

        rospy.loginfo('%s: Grasping handle...' % (self._action_name))
        self.whole_body.move_end_effector_pose(geometry.pose(x=handle_pose.x, y=handle_pose.y,  z=handle_pose.z-0.05), 'head_rgbd_sensor_rgb_frame')
        try:
            self.whole_body.move_end_effector_pose(geometry.pose(z=0.02), 'hand_palm_link')
        except:
            rospy.loginfo("%s: Couldn't move forward..." % (self._action_name))
            pass
        self.gripper.apply_force(self._GRASP_FORCE)
        rospy.sleep(2)

        rospy.loginfo('%s: Executing opening motion...' % (self._action_name))
        self.whole_body.move_end_effector_pose(geometry.pose(y=0.04), 'hand_palm_link')
        rospy.sleep(2)

        self.omni_base.go_rel(-0.1,0,0)
        self.whole_body.move_end_effector_pose(geometry.pose(z=-0.1), 'hand_palm_link')
        self.gripper.set_distance(0.1)

        self.whole_body.move_to_go()

        rospy.loginfo('%s: Succeeded door opening. Now returning results.' % self._action_name)
        result = OpenDoorResult()
        result.result = True
        self._as.set_succeeded(result)


if __name__ == '__main__':
    rospy.init_node('open_door_server_node')
    server = OpenDoorAction(rospy.get_name())
    rospy.spin()
