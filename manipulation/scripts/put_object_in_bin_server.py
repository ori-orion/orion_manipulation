#! /usr/bin/env python3

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


class PutObjectInBinAction(object):

    def __init__(self, name):
        self._action_name = 'put_object_in_bin'
        self._as = actionlib.SimpleActionServer(self._action_name, 	orion_actions.msg.PutObjectInBinAction,
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

        self.tts.say("I will put the object in the bin.")
        rospy.sleep(1)

        try:

            try:
                # Move gripper above bin
                self.move_above_bin()
            except:
                # Move gripper above bin
                rospy.loginfo('%s: Encountered an error. Trying again.' % self._action_name)
                self.move_above_bin()


            self.omni_base.go_rel(0, -0.06, 0)
            self.omni_base.go_rel(0.32, 0, 0)

            # Move grasper down
            rospy.loginfo('%s: Lowering gripper.' % self._action_name)
            self.whole_body.move_end_effector_pose(geometry.pose(z=0.30),'hand_palm_link')
            self.omni_base.go_rel(0.05, 0, 0)
            self.whole_body.move_to_joint_positions({'wrist_flex_joint': -1.3})


            self.tts.say("Dropping object into the bin.")
            rospy.sleep(1)
            rospy.loginfo('%s: Opening gripper.' % self._action_name)
            self.gripper.set_distance(1)

            # Return to "neutral" pose
            rospy.loginfo('%s: Returning to neutral pose.' % (self._action_name))
            self.tts.say("Returning to neutral position.")
            rospy.sleep(1)
            self.whole_body.move_to_neutral()


            rospy.loginfo('%s: Succeeded' % self._action_name)
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
    rospy.init_node('put_object_in_bin_server')
    server = PickUpBinBagAction(rospy.get_name())
    rospy.spin()
