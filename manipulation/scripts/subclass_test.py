#! /usr/bin/env python3
""" Action server for picking up objects.
Variable self.use_grasp_synthesis can be used to change between using grasp synthesis to determine grasp position and
using a hard-coded 5cm away horizontal pre-grasp pose. grasp synthesis uses point cloud segmentation and gdp grasp
synthesis
"""
__author__ = "Mark Finean"
__email__ = "mfinean@robots.ox.ac.uk"

import hsrb_interface
import rospy
import actionlib
import numpy as np
import tf
import hsrb_interface.geometry as geometry
import tf.transformations as T
import math
import cv2
import imutils
import message_filters

from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError

from manipulation.manipulation_header import CollisionMapper
from actionlib_msgs.msg import GoalStatus
from tmc_suction.msg import (SuctionControlAction, SuctionControlGoal)
from tmc_manipulation_msgs.msg import CollisionObject
from orion_actions.msg import *

from manipulation.pick_up_class import PickUpBase

from hsrb_interface import robot as _robot
_robot.enable_interactive()

# For grasp synthesis
from point_cloud_filtering.srv import SegmentObject
from gpd.msg import GraspConfigList


def analyse_hand_image(hand_cam_topic, height_above_object):

    camera_matrix = np.array([[554.3827128226441, 0.0, 320.5], [0.0, 554.3827128226441, 240.5], [0.0, 0.0, 1.0]])
    inv_mat = np.linalg.inv(camera_matrix)

    rgb_sub = message_filters.Subscriber(hand_cam_topic, Image, queue_size=1, buff_size=2**24)
    image_msg = rospy.wait_for_message(hand_cam_topic, Image)
    bridge = CvBridge()
    img_in = bridge.imgmsg_to_cv2(image_msg, desired_encoding="bgr8")


    mean_0 = img_in[:, :, 0].mean()
    mean_1 = img_in[:, :, 1].mean()
    mean_2 = img_in[:, :, 2].mean()

    image_cp = img_in
    image_cp[:, :, 0][image_cp[:, :, 0] >= mean_0 - 30] = 255
    image_cp[:, :, 1][image_cp[:, :, 1] >= mean_1 - 30] = 255
    image_cp[:, :, 2][image_cp[:, :, 2] >= mean_2 - 30] = 255

    mask = np.logical_and(np.logical_and(image_cp[:, :, 0] < 255, image_cp[:, :, 1] < 255), image_cp[:, :, 2] < 255)
    coords = np.where(mask)

    y_coords = coords[0] # y coord
    x_coords = coords[1] # x coord

    mean_y = y_coords.mean()
    mean_x = x_coords.mean()

    x_len = np.shape(img_in)[1]
    y_len = np.shape(img_in)[0]

    # off_y_px = mean_y - y_len / 2.0  # if positive need to go lower
    # off_x_px = mean_x - x_len / 2.0  # if positive need to move right

    y_coords_no_mean = y_coords - y_coords.mean()
    x_coords_no_mean = x_coords - x_coords.mean()

    inds = np.where(x_coords_no_mean > 0)

    y_coords_no_mean_pos = y_coords_no_mean[inds]
    x_coords_no_mean_pos = x_coords_no_mean[inds]

    theta = np.mean(np.arctan(np.divide(-y_coords_no_mean_pos.astype(float), x_coords_no_mean_pos.astype(float))) * 180 / math.pi)

    X = np.array([[mean_x], [mean_y], [1]])
    distance_to_move = np.matmul(inv_mat, X) * height_above_object

    return distance_to_move[0][0], distance_to_move[1][0], theta



class PickUpObjectAction(PickUpBase):

    def __init__(self, name):
        super(PickUpObjectAction, self).__init__(name)

        self._action_name = 'pick_up_object'
        rospy.loginfo('%s: Action name is: %s' % (self._action_name, name))

        self._as = actionlib.SimpleActionServer(self._action_name, 	orion_actions.msg.PickUpObjectAction,
                                                execute_cb=self.execute_cb, auto_start=False)
        self._as.start()

        self.collision_mapper = CollisionMapper(self.robot)

        # Set up publisher for the collision map
        self.pub = rospy.Publisher('known_object', CollisionObject, queue_size=1)
        self.goal_pose_br = tf.TransformBroadcaster()

        self.grasps = None
        self.use_grasp_synthesis = False
        self.small_objects = ['fork', 'knife', 'spoon']

        self.hand_cam_topic = '/hsrb/hand_camera/image_raw'

        self.goal_object = None
        rospy.loginfo('%s: Initialised. Ready for clients.' % self._action_name)

    def collision_mod(self, msg):
        if self.goal_object:
            object_pose = self.get_object_pose(self.goal_object)
            upper_bound = object_pose + np.array([0.7, 0.7, 1])
            lower_bound = object_pose - np.array([0.7, 0.7, 1])

            object_upper_bound = object_pose + np.array([0.05, 0.05, 0.03])
            object_lower_bound = object_pose - np.array([0.05, 0.05, 0.03])

            # Get the message
            message = msg
            rospy.loginfo('%s: Removing excess collision space.' % self._action_name)

            # Find which boxes to removes
            inds_to_remove = []
            for i in range(len(message.poses)):
                pose = message.poses[i]
                pose_arr = np.array([pose.position.x,pose.position.y,pose.position.z])

                # remove excess environment
                if not(np.all(pose_arr <= upper_bound) and np.all(pose_arr >= lower_bound)):
                    inds_to_remove.append(i)

                # approx remove the object
                if np.all(pose_arr <= object_upper_bound) and np.all(object_lower_bound >= lower_bound):
                    inds_to_remove.append(i)

            # Remove the boxes
            for index in sorted(inds_to_remove, reverse=True):
                del message.poses[index], message.shapes[index]

            # Publish the filtered message
            self.pub.publish(message)
        else:
            self.pub.publish(msg)

    def collision_callback(self, msg):
        self.collision_msg = msg

    def grab_object(self, chosen_pregrasp_pose, chosen_grasp_pose):
        self.whole_body.end_effector_frame = 'hand_palm_link'

        rospy.loginfo('%s: Opening gripper.' % (self._action_name))
        self.gripper.command(1.2)
        self.whole_body.collision_world = self.collision_world

        try:
            if self.use_grasp_synthesis:
                # Segment the object point cloud first
                object_position_head_frame = self.get_head_frame_object_pose(self.goal_object)

                # Call segmentation (lasts 10s)
                seg_response = self.segment_object(object_position_head_frame)

                self.tts.say('I am trying to calculate the best possible grasp position')
                rospy.sleep(1)
                # Get the best grasp - returns the pose-tuple in the head-frame
                grasp = self.get_grasp()

                rospy.loginfo('%s: Moving to pre-grasp position.' % (self._action_name))
                self.tts.say('I will now move into the grasp position')
                rospy.sleep(1)
                self.whole_body.move_end_effector_pose(grasp, 'head_rgbd_sensor_rgb_frame')

            else:
                # Move to pregrasp
                rospy.loginfo('%s: Calculating grasp pose.' % (self._action_name))
                self.tts.say('I am now calculating the grasp position')
                rospy.sleep(1)

                goal_pose = self.get_goal_pose(relative=chosen_pregrasp_pose)

                self.goal_pose_br.sendTransform((goal_pose.pos.x, goal_pose.pos.y, goal_pose.pos.z),
                                                (goal_pose.ori.x, goal_pose.ori.y, goal_pose.ori.z, goal_pose.ori.w),
                                                rospy.Time.now(),
                                                'goal_pose',
                                                'odom')

                rospy.loginfo('%s: Moving to pre-grasp position.' % (self._action_name))
                self.tts.say('Moving to pre-grasp position.')
                rospy.sleep(1)
                self.whole_body.move_end_effector_pose(goal_pose, 'odom')

            # Turn off collision checking to get close and grasp
            self.tts.say('Turning off collision checking to get closer.')
            rospy.sleep(1)
            rospy.loginfo('%s: Turning off collision checking to get closer.' % (self._action_name))
            self.whole_body.collision_world = None
            rospy.sleep(1)

            # Move to grasp pose
            rospy.loginfo('%s: Moving to grasp position.' % (self._action_name))
            self.tts.say('Moving to grasp position.')
            rospy.sleep(1)
            self.whole_body.move_end_effector_pose(chosen_grasp_pose, self.whole_body.end_effector_frame)

            # Specify the force to grasp
            self.tts.say('Grasping object.')
            rospy.sleep(1)
            self.gripper.apply_force(self._GRASP_FORCE)
            rospy.loginfo('%s: Object grasped.' % self._action_name)

            return True

        except Exception as e:
            rospy.loginfo('{0}: Encountered exception {1}.'.format(self._action_name, str(e)))
            self.whole_body.collision_world = None
            rospy.sleep(1)
            rospy.loginfo('%s: Returning to neutral pose.' % (self._action_name))
            self.whole_body.move_to_neutral()
            self._as.set_aborted()
            return False

    def suck_object(self):
        self.whole_body.end_effector_frame = 'hand_l_finger_vacuum_frame'

        rospy.loginfo('%s: Closing gripper.' % self._action_name)
        self.gripper.command(0.1)

        rospy.loginfo('%s: Turning on on the suction...' % self._action_name)

        # Create action client to control suction
        suction_action = '/hsrb/suction_control'
        suction_control_client = actionlib.SimpleActionClient(suction_action, SuctionControlAction)

        # Wait for connection
        try:
            if not suction_control_client.wait_for_server(rospy.Duration(self._CONNECTION_TIMEOUT)):
                raise Exception(suction_action + ' does not exist')
        except Exception as e:
            rospy.logerr(e)

        # Send a goal to start suction
        rospy.loginfo('%s: Suction server found. Activating suction...' % self._action_name)
        suction_on_goal = SuctionControlGoal()
        suction_on_goal.timeout = self._SUCTION_TIMEOUT
        suction_on_goal.suction_on.data = True

        if (suction_control_client.send_goal_and_wait(suction_on_goal) == GoalStatus.SUCCEEDED):
            rospy.loginfo('Suction succeeded. Object picked up')
        else:
            rospy.loginfo('Suction failed')
            return

        self.whole_body.collision_world = self.collision_world
        self.whole_body.move_end_effector_pose(geometry.pose(z=0.05, ei=3.14), self.goal_object)

        # Turn off collision checking to get close and grasp
        rospy.loginfo('%s: Turning off collision checking to get closer.' % (self._action_name))
        self.whole_body.collision_world = None
        self.whole_body.move_end_effector_pose(geometry.pose(z=0.045), self.goal_object)

    def grab_small_object(self, chosen_pregrasp_pose):
        self.whole_body.end_effector_frame = 'hand_palm_link'
        height_above_object = 0.07

        rospy.loginfo('%s: Opening gripper.' % (self._action_name))
        self.gripper.command(1.2)
        self.whole_body.collision_world = self.collision_world

        # Move to pregrasp
        rospy.loginfo('%s: Calculating grasp pose.' % (self._action_name))
        self.tts.say('I am now calculating the grasp position')
        rospy.sleep(1)

        goal_pose = self.get_goal_pose()

        # Place hand 7cm above the small object
        self.goal_pose_br.sendTransform((goal_pose.pos.x, goal_pose.pos.y, goal_pose.pos.z + height_above_object),
                                        (goal_pose.ori.x, goal_pose.ori.y, goal_pose.ori.z, goal_pose.ori.w),
                                        rospy.Time.now(),
                                        'goal_pose',
                                        'odom')

        rospy.loginfo('%s: Moving to pre-grasp position.' % (self._action_name))
        self.tts.say('Moving to pre-grasp position.')
        rospy.sleep(1)
        self.whole_body.move_end_effector_pose(goal_pose, 'odom')

        self.tts.say("I am now using the hand camera to align the object.")

        for i in range(3):
            x_to_move, y_to_move, theta = analyse_hand_image(self.hand_cam_topic, height_above_object)
            theta_rad = theta * math.pi / 180.0
            rospy.loginfo('{0}: Need to rotate "{1:.2f}" degrees.'.format(self._action_name, theta))
            self.tts.say('I will rotate "{1:.2f}" degrees.'.format(theta))

            self.whole_body.move_end_effector_pose(geometry.pose(x=x_to_move), 'hand_palm_link')
            rospy.sleep(1)
            self.whole_body.move_end_effector_pose(geometry.pose(y=y_to_move), 'hand_palm_link')
            rospy.sleep(1)
            self.whole_body.move_end_effector_pose(geometry.pose(ek=-theta_rad), 'hand_palm_link')
            rospy.sleep(1)


        # Move to grasp pose
        rospy.loginfo('%s: Moving to grasp position.' % (self._action_name))
        self.tts.say('Moving to grasp position.')
        rospy.sleep(1)
        self.whole_body.move_end_effector_pose(geometry.pose(z=0.02), self.whole_body.end_effector_frame)

        # Specify the force to grasp
        self.tts.say('Grasping object.')
        rospy.sleep(1)
        self.gripper.apply_force(self._GRASP_FORCE)
        rospy.loginfo('%s: Object grasped.' % self._action_name)

        return True


    def execute_cb(self, goal_msg):
        _result = PickUpObjectResult()
        is_preempted = False

        # Currently doesn't do anything other than relay to another topic
        rospy.Subscriber("known_object_pre_filter", CollisionObject, self.collision_callback)

        goal_tf_in = goal_msg.goal_tf
        goal_tf = None

        while goal_tf is None:
            goal_tf = self.get_similar_tf(goal_tf_in)

            if self._as.is_preempt_requested():
                rospy.loginfo('%s: Preempted. Moving to go and exiting.' % self._action_name)
                self.whole_body.move_to_go()
                self._as.set_preempted()
                is_preempted = True
                return


            if goal_tf is None:
                rospy.loginfo('{0}: Found no similar tf frame. Trying again'.format(self._action_name))

        if is_preempted:
            return

        # Found the goal tf so proceed to pick up
        rospy.loginfo('{0}: Choosing tf frame "{1}".'.format(self._action_name, str(goal_tf)))
        self.set_goal_object(goal_tf)
        obj_dist = self.get_object_distance(goal_tf)
        rospy.loginfo('{0}: Distance to object is "{1:.2f}"m.'.format(self._action_name, obj_dist))
        self.tts.say('I can see the object and it is "{:.2f}" metres away.'.format(obj_dist))
        rospy.sleep(1)

        if self.goal_object == 'postcard':
            grasp_type = 'suction'
        elif self.goal_object in self.small_objects:
            grasp_type = 'small_object_grab'
            chosen_pregrasp_pose = geometry.pose(z=0.07, ej=-math.pi)
        else:
            grasp_type = 'grab'
            chosen_pregrasp_pose = geometry.pose(z=-0.08, ek=-math.pi/2)
            chosen_grasp_pose = geometry.pose(z=0.08)

        # ------------------------------------------------------------------------------
        # Check the object is in sight
        self.check_for_object(self.goal_object)

        # Look at the object - this is to make sure that we get all of the necessary collision map
        rospy.loginfo('%s: Moving head to look at the object.' % self._action_name)
        self.whole_body.gaze_point(ref_frame_id=self.goal_object)

        # Set collision map
        self.tts.say("I am now evaluating my environment so that I don't collide with anything.")
        rospy.sleep(1)
        rospy.loginfo('%s: Getting Collision Map.' % self._action_name)
        self.collision_mapper.get_collision_map()
        rospy.loginfo('%s: Collision Map generated.' % self._action_name)

        rospy.loginfo('%s: Pruning the collision map.' % self._action_name)
        self.collision_mod(self.collision_msg)

        # Get the object pose to subtract from collision map
        # self.check_for_object(goal_tf)

        # Give opportunity to preempt
        if self._as.is_preempt_requested():
            self.tts.say("I was preempted. Moving to go.")
            rospy.loginfo('%s: Preempted. Moving to go and exiting.' % self._action_name)
            self.whole_body.move_to_go()
            self._as.set_preempted()
            return

        if grasp_type == 'suction':
            self.suck_object()
        elif grasp_type == 'small_object_grab':
            self.tts.say("I will now pick up the small object")
            rospy.sleep(1)
            grab_success = self.grab_small_object(chosen_pregrasp_pose)
        else:
            self.tts.say("I will now pick up the object")
            rospy.sleep(1)
            grab_success = self.grab_object(chosen_pregrasp_pose, chosen_grasp_pose)

        if grab_success == False:
            self.whole_body.move_to_go()
            return

        self.tts.say("Object grasped successfully. Now returning to normal position.")
        rospy.sleep(1)

        # Now return to moving position
        success = self.finish_position()

        if success:
            rospy.loginfo('%s: Succeeded' % self._action_name)
            _result.result = True
            self._as.set_succeeded(_result)
        else:
            rospy.loginfo('%s: Failed' % self._action_name)
            _result.result = False
            self._as.set_aborted()


if __name__ == '__main__':
    rospy.init_node('pick_up_object_server_node')
    server = PickUpObjectAction(rospy.get_name())
    rospy.spin()
