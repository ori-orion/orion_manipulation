#! /usr/bin/env python

import hsrb_interface
import rospy
import actionlib
import tf
import tf.transformations as T
import math
import hsrb_interface.geometry as geometry
from hsrb_interface import robot as _robot

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

        rospy.loginfo('%s: Initialised. Ready for clients.' % self._action_name)

    def move_above_bin(self):
        self.tts.say("I will pick up this bin bag.")
        rospy.sleep(1)

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

        # Start the force sensor capture
        force_sensor_capture = ForceSensorCapture()

        try:
            self.omni_base.go_rel(0, -0.15, 0)
            self.omni_base.go_rel(0.17, 0, 0)
        except:
            pass

        try:

            try:
                # Move gripper above bin
                self.move_above_bin()
            except:
                # Move gripper above bin
                rospy.loginfo('%s: Encountered an error. Trying again.' % self._action_name)
                self.move_above_bin()

            # Get initial data of force sensor
            pre_grasp_force_list = force_sensor_capture.get_current_force()

            # Move grasper down
            rospy.loginfo('%s: Lowering gripper.' % self._action_name)
            self.whole_body.move_end_effector_pose(geometry.pose(z=0.60),'hand_palm_link')
            
            self.tts.say("Grasping the bin bag.")
            rospy.sleep(1)
            rospy.loginfo('%s: Closing gripper.' % self._action_name)
            self.gripper.apply_force(2.0)
            
            # Lift
            rospy.loginfo('%s: Lifting bin bag up.' % self._action_name)
            self.tts.say("Lifting bin bag.")
            rospy.sleep(1)
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
            rospy.loginfo('%s: Returning to neural pose.' % (self._action_name))
            self.tts.say("Returning to neutral position.")
            rospy.sleep(1)
            self.whole_body.move_to_neutral()

            # Return to "go" pose
            rospy.loginfo('%s: Moving bin higher to avoid the laser.' % (self._action_name))
            self.tts.say("Moving bin higher to avoid the laser.")
            rospy.sleep(1)
            self.whole_body.move_end_effector_pose(geometry.pose(x=0.5),'hand_palm_link')

            self.whole_body.linear_weight = 3

            rospy.loginfo('%s: Succeeded' % self._action_name)
            self.tts.say("Succeeded in pick up.")
            rospy.sleep(1)
            _result.result = True
            self._as.set_succeeded(_result)

        except Exception as e:
            rospy.loginfo('{0}: Encountered exception {1}.'.format(self._action_name, str(e)))
            self.tts.say("I encountered a problem. Returning to go position.")
            rospy.sleep(2)
            rospy.loginfo('%s: Returning to go pose.' % (self._action_name))
            self.whole_body.move_to_go()
            self._as.set_aborted()


if __name__ == '__main__':
    rospy.init_node('pick_up_bin_bag_server')
    server = PickUpBinBagAction(rospy.get_name())
    rospy.spin()
