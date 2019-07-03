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
        self.gripper = self.robot.try_get('gripper')
        self.whole_body.end_effector_frame = 'hand_palm_link'
        self.whole_body.looking_hand_constraint = True
        self.tts = self.robot.try_get('default_tts')
        self.tts.language = self.tts.ENGLISH

        self.whole_body.planning_timeout = 20.0  # Increase planning timeout. Default is 10s

        rospy.loginfo('%s: Initialised. Ready for clients.' % self._action_name)

    def execute_cb(self, goal_msg):
        # Messages for feedback / results
        _result = PickUpBinBagResult()
        _result.result = False

        is_preempted = False

        if is_preempted:
            return

        try:

            self.tts.say("I will pick up this bin bag.")
            rospy.sleep(1)
    
            rospy.loginfo('%s: Moving to go position.' % self._action_name)
            self.whole_body.move_to_go()
            rospy.sleep(1)

            self.whole_body.linear_weight = 100

            rospy.loginfo('%s: Moving end effector above bin.' % self._action_name)
            
            # Move grasper over the object to pick up
            self.whole_body.move_end_effector_pose(geometry.pose(x=0.4,z=1.0,ei=math.pi),'base_footprint')
         
            # Move grasper down
            rospy.loginfo('%s: Lowering gripper.' % self._action_name)
            self.whole_body.move_end_effector_pose(geometry.pose(z=0.4),'hand_palm_link')   
            
            self.tts.say("Grasping the bin bag.")
            rospy.sleep(1)
            rospy.loginfo('%s: Closing gripper.' % self._action_name)
            self.gripper.apply_force(2.0)
            
            # Move to bin bag and close gripper
            rospy.loginfo('%s: Lifting bin bag up.' % self._action_name)
            self.tts.say("Lifting bin bag.")
            rospy.sleep(1)
            self.whole_body.move_end_effector_pose(geometry.pose(z=-0.4),'hand_palm_link') 
            
            # Return to "go" pose 
            rospy.loginfo('%s: Returning to go pose.' % (self._action_name))
            self.tts.say("Returning to go position.")
            rospy.sleep(1)
            self.whole_body.move_to_go()

            self.whole_body.linear_weight = 50

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
    rospy.init_node('pick_up_bin_bag_server')
    server = PickUpBinBagAction(rospy.get_name())
    rospy.spin()
