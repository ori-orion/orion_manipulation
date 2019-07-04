#! /usr/bin/env python

import hsrb_interface
import rospy
import actionlib
import tf
import tf.transformations as T
import math
import hsrb_interface.geometry as geometry
from hsrb_interface import robot as _robot

import cv2
import imutils
import message_filters
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

_robot.enable_interactive()

from actionlib_msgs.msg import GoalStatus
from orion_actions.msg import *

from geometry_msgs.msg import WrenchStamped


def compute_difference(pre_data_list, post_data_list):
    if len(pre_data_list) != len(post_data_list):
        raise ValueError('Argument lists differ in length')

    # Calculate square sum of difference
    square_sums = sum([math.pow(b - a, 2)
                       for (a, b) in zip(pre_data_list, post_data_list)])

    return math.sqrt(square_sums)

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


class ForceSensorCapture(object):
    """Subscribe and hold force sensor data - Copyright (C) 2016 Toyota Motor Corporation"""

    def __init__(self):
        self._force_data_x = 0.0
        self._force_data_y = 0.0
        self._force_data_z = 0.0

        # Subscribe force torque sensor data
        ft_sensor_topic = '/hsrb/wrist_wrench/raw'
        self._wrist_wrench_sub = rospy.Subscriber(
            ft_sensor_topic, WrenchStamped, self.__hand_force_sensor_cb)

        # Wait for connection
        try:
            rospy.wait_for_message(ft_sensor_topic, WrenchStamped,
                                   timeout=10)
        except Exception as e:
            rospy.logerr(e)

    def get_current_force(self):
        return [self._force_data_x, self._force_data_y, self._force_data_z]

    def __hand_force_sensor_cb(self, data):
        self._force_data_x = data.wrench.force.x
        self._force_data_y = data.wrench.force.y
        self._force_data_z = data.wrench.force.z


class PickUpBinBagAction(object):

    def __init__(self, name):
        self._action_name = 'pick_up_bin_bag'
        self._as = actionlib.SimpleActionServer(self._action_name, 	orion_actions.msg.PickUpBinBagAction,
                                                execute_cb=self.execute_cb, auto_start=False)
        self._as.start()
        rospy.loginfo('%s: Action name is: %s' % (self._action_name, name))

        # Preparation for using the robot functions
        self.robot = hsrb_interface.Robot()
        self.whole_body = self.robot.try_get('whole_body')
        self.omni_base = self.robot.try_get('omni_base')
        self.gripper = self.robot.try_get('gripper')
        self.whole_body.end_effector_frame = 'hand_palm_link'
        self.tts = self.robot.try_get('default_tts')
        self.tts.language = self.tts.ENGLISH

        self.whole_body.planning_timeout = 20.0  # Increase planning timeout. Default is 10s

        self.counter = 0
        self.tried_bin_lid = False
        self.hand_cam_topic = '/hsrb/hand_camera/image_raw'

        rospy.loginfo('%s: Initialised. Ready for clients.' % self._action_name)

    def pick_up_bin_lid(self):
        self.omni_base.go_rel(0, -0.07, 0)
        self.omni_base.go_rel(0.32, 0, 0)

        self.tried_bin_lid = True

        grasp_success_bool = False

        try:
            # Start the force sensor capture
            force_sensor_capture = ForceSensorCapture()

            rospy.loginfo('%s: Opening gripper.' % self._action_name)
            self.gripper.set_distance(1.0)

            self.tts.say("I will pick up the bin lid.")
            rospy.sleep(1)

            try:
                # Move gripper above bin
                self.move_above_bin()
            except:
                # Move gripper above bin
                rospy.loginfo('%s: Encountered an error. Trying again.' % self._action_name)
                self.move_above_bin()


            # Move grasper down
            rospy.loginfo('%s: Lowering gripper.' % self._action_name)
            self.whole_body.move_end_effector_pose(geometry.pose(z=0.30),'hand_palm_link')
            height_above_object = 0.235

            # Get initial data of force sensor
            pre_grasp_force_list = force_sensor_capture.get_current_force()

            for i in range(3):
                x_to_move, y_to_move, theta = analyse_hand_image(self.hand_cam_topic, height_above_object)
                theta_rad = theta * math.pi / 180.0
                rospy.loginfo('{:}: Need to rotate "{:.2f}" degrees.'.format(self._action_name, theta))
                self.tts.say('I will rotate "{:.2f}" degrees.'.format(theta))

                self.whole_body.move_end_effector_pose(geometry.pose(x=x_to_move), 'hand_palm_link')
                rospy.sleep(1)
                self.whole_body.move_end_effector_pose(geometry.pose(y=y_to_move), 'hand_palm_link')
                rospy.sleep(1)
                self.whole_body.move_end_effector_pose(geometry.pose(ek=-theta_rad), 'hand_palm_link')
                rospy.sleep(1)

            self.whole_body.move_end_effector_pose(geometry.pose(z=height_above_object-0.08),'hand_palm_link')

            self.tts.say("Grasping the bin lid.")
            rospy.sleep(1)
            rospy.loginfo('%s: Closing gripper.' % self._action_name)
            self.gripper.apply_force(2.0)

            self.whole_body.move_end_effector_pose(geometry.pose(z=-height_above_object+0.08),'hand_palm_link')

            post_grasp__force_list = force_sensor_capture.get_current_force()

            # Get the weight of the object and convert newtons to grams
            force_difference = compute_difference(pre_grasp_force_list, post_grasp__force_list)
            weight = math.floor(force_difference / 9.81 * 1000)

            print "The weight is " + str(weight) + 'grams.'
            rospy.loginfo('{0}: The weight in grams is {1}.'.format(self._action_name, str(weight)))
            self.tts.say('{0}: I can feel a weight of {1} grams.'.format(self._action_name, str(weight)))
            rospy.sleep(3)

            if weight > 300:
                rospy.loginfo('%s: Handle grasped successfully.' % self._action_name)
                self.tts.say("Handle grasped successfully.")
                rospy.sleep(1)
                self.omni_base.go_rel(0, 0, -math.pi/8)

                if self.counter > 0:
                    self.whole_body.move_end_effector_pose(geometry.pose(z=-0.1),'hand_palm_link')

                rospy.loginfo('%s: Opening gripper.' % self._action_name)
                self.gripper.set_distance(1.0)
                self.omni_base.go_rel(0, 0, math.pi/8)

                if self.counter > 0:
                    self.whole_body.move_end_effector_pose(geometry.pose(z=0.1),'hand_palm_link')

                grasp_success_bool = True
            else:
                rospy.loginfo('%s: Failed to grasp handle.' % self._action_name)
                self.tts.say("Failed to grasp handle.")
                rospy.sleep(1)
                grasp_success_bool = False
                rospy.loginfo('%s: Encountered a problem.' % self._action_name)
                self.tts.say("Encountered a problem. Please could you remove the bin lid for me.")
                rospy.sleep(10)
                self.whole_body.move_to_neutral()
        except Exception as e:
            rospy.loginfo('{0}: Encountered exception {1}.'.format(self._action_name, str(e)))
            self.whole_body.move_to_neutral()
            self.tts.say("Encountered a problem. Please could you remove the bin lid for me.")
            rospy.sleep(10)

        return grasp_success_bool


    def move_above_bin(self):

            rospy.loginfo('%s: Moving to go position.' % self._action_name)
            try:
                self.whole_body.move_to_go()
            except:
                pass

            rospy.loginfo('%s: Opening gripper.' % self._action_name)
            self.gripper.set_distance(1.0)

            rospy.loginfo('%s: Changing linear weight.' % self._action_name)
            self.whole_body.linear_weight = 100

            rospy.loginfo('%s: Moving end effector above bin.' % self._action_name)
            self.tts.say("Moving end effector above bin.")
            rospy.sleep(1)

            # Move grasper over the object to pick up
            self.whole_body.move_end_effector_pose(geometry.pose(x=0.4,z=1.0,ei=math.pi),'base_footprint')

    def execute_cb(self, goal_msg):
        # Messages for feedback / results
        _result = PickUpBinBagResult()
        _result.result = False

        removed_bin_lid_bool = False

        # Start the force sensor capture
        force_sensor_capture = ForceSensorCapture()

        # removed_bin_lid_bool = self.pick_up_bin_lid()

        self.tts.say("I will pick up this bin bag.")
        rospy.sleep(1)

        try:

            if not removed_bin_lid_bool:
                try:
                    # Move gripper above bin
                    self.move_above_bin()
                except:
                    # Move gripper above bin
                    rospy.loginfo('%s: Encountered an error. Trying again.' % self._action_name)
                    self.move_above_bin()

            if self.tried_bin_lid == False:
                self.omni_base.go_rel(0, -0.06, 0)
                self.omni_base.go_rel(0.32, 0, 0)

            # if self.counter < 0:
            #     try:
            #         self.omni_base.go_rel(0, -0.10, 0)
            #         self.omni_base.go_rel(0.25, 0, 0)
            #     except:
            #         pass




            # Move grasper down
            rospy.loginfo('%s: Lowering gripper.' % self._action_name)
            self.whole_body.move_end_effector_pose(geometry.pose(z=0.30),'hand_palm_link')
            self.omni_base.go_rel(0.05, 0, 0)
            self.whole_body.move_to_joint_positions({'wrist_flex_joint': -1.3})
            self.whole_body.move_to_joint_positions({'arm_flex_joint': -2.0})
            self.whole_body.move_to_joint_positions({'wrist_flex_joint': -1.0})
            self.whole_body.move_end_effector_pose(geometry.pose(z=0.10),'hand_palm_link')


            # Get initial data of force sensor
            pre_grasp_force_list = force_sensor_capture.get_current_force()

            self.tts.say("Grasping the bin bag.")
            rospy.sleep(1)
            rospy.loginfo('%s: Closing gripper.' % self._action_name)
            self.gripper.apply_force(2.0)
            
            # Lift
            rospy.loginfo('%s: Lifting bin bag up.' % self._action_name)
            self.tts.say("Lifting bin bag.")
            rospy.sleep(1)
            self.whole_body.move_to_joint_positions({'arm_flex_joint': -math.pi/2})
            self.whole_body.move_end_effector_pose(geometry.pose(z=-0.60),'hand_palm_link')

            post_grasp__force_list = force_sensor_capture.get_current_force()

            # Get the weight of the object and convert newtons to grams
            force_difference = compute_difference(pre_grasp_force_list, post_grasp__force_list)
            weight = math.floor(force_difference / 9.81 * 1000)

            print "The weight is " + str(weight) + 'grams.'
            rospy.loginfo('{0}: The weight in grams is {1}.'.format(self._action_name, str(weight)))
            self.tts.say('{0}: I can feel a weight of {1} grams.'.format(self._action_name, str(weight)))
            rospy.sleep(3)

            if weight < 100:
                rospy.loginfo('{0}: No object picked up.'.format(self._action_name))
                self.tts.say("Grasp unsuccessful. Returning to go.")
                rospy.sleep(1)
                rospy.loginfo('%s: Returning to go pose.' % (self._action_name))
                self.whole_body.move_to_go()
                self._as.set_aborted()
                return

            # Return to "neutral" pose
            rospy.loginfo('%s: Returning to neutral pose.' % (self._action_name))
            self.tts.say("Returning to neutral position.")
            rospy.sleep(1)
            self.whole_body.move_to_neutral()

            # Return to "go" pose
            rospy.loginfo('%s: Moving bin higher to avoid the laser.' % (self._action_name))
            self.tts.say("Moving bin higher to avoid the laser.")
            rospy.sleep(1)
            self.whole_body.move_end_effector_pose(geometry.pose(x=0.5),'hand_palm_link')
            self.whole_body.move_to_joint_positions({'arm_flex_joint': -0.05})

            self.whole_body.linear_weight = 3

            rospy.loginfo('%s: Succeeded' % self._action_name)
            self.tts.say("Succeeded in pick up.")
            self.counter+=1
            self.tried_bin_lid = False

            rospy.sleep(1)
            _result.result = True
            self._as.set_succeeded(_result)

        except Exception as e:
            rospy.loginfo('{0}: Encountered exception {1}.'.format(self._action_name, str(e)))
            self.tts.say("I encountered a problem. Returning to go position.")
            rospy.sleep(2)
            rospy.loginfo('%s: Opening gripper.' % self._action_name)
            self.gripper.set_distance(1.0)

            rospy.loginfo('%s: Returning to go pose.' % (self._action_name))
            self.whole_body.move_to_go()
            self.counter+=1
            self.tried_bin_lid = False
            self._as.set_aborted()


if __name__ == '__main__':
    rospy.init_node('pick_up_bin_bag_server')
    server = PickUpBinBagAction(rospy.get_name())
    rospy.spin()
