#! /usr/bin/env python
""" Action server for putting objects on a surface in front.
Currently in development
"""

__author__ = "Mark Finean"
__email__ = "mfinean@robots.ox.ac.uk"

import hsrb_interface
import rospy
import actionlib
import hsrb_interface.geometry as geometry
from hsrb_interface import robot as _robot

_robot.enable_interactive()

from actionlib_msgs.msg import GoalStatus
from orion_actions.msg import *


class PutObjectOnSurfaceAction(object):

    def __init__(self, name):
        self._action_name = 'put_object_on_surface'
        self._as = actionlib.SimpleActionServer(self._action_name, 	orion_actions.msg.PutObjectOnSurfaceAction,
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
        _result = PutObjectOnSurfaceResult()
        _result.result = False

        # Give opportunity to preempt
        if self._as.is_preempt_requested():
            rospy.loginfo('%s: Preempted' % self._action_name)
            self._as.set_preempted()
            return

        try:
            rospy.loginfo('%s: Placing gripper close to surface in front' % (self._action_name))
            self.tts.say("I will place the object on the surface in front of me.")
            rospy.sleep(2)
            self.whole_body.move_to_neutral()
            rospy.sleep(1)

            # Place object of surface
            ## NEED TO CHANGE THE GOAL LOCATION HERE
            self.whole_body.move_end_effector_pose(geometry.pose(x=-0.6, y=0, z=0.2), 'hand_palm_link')

            # Let go of the object
            rospy.sleep(1)
            rospy.loginfo('%s: Opening gripper.' % (self._action_name))
            self.gripper.command(1.2)
            self.tts.say("Object placed successfully. Returning to go position.")
            rospy.sleep(2)

            # Move the gripper back a bit then return to go
            self.whole_body.linear_weight = 100
            self.whole_body.move_end_effector_pose(geometry.pose(z=-0.2), 'hand_palm_link')
            rospy.loginfo('%s: Returning to go pose.' % (self._action_name))
            self.whole_body.move_to_go()

            rospy.loginfo('%s: Succeeded' % self._action_name)
            _result.result = True
            self._as.set_succeeded(_result)

        except Exception as e:
            rospy.loginfo('{0}: Encountered exception {1}.'.format(self._action_name, str(e)))
            self.tts.say("I encountered a problem. Returning to go position and aborting placement.")
            rospy.sleep(2)
            rospy.loginfo('%s: Returning to go pose.' % (self._action_name))
            self.whole_body.move_to_go()
            self._as.set_aborted()






if __name__ == '__main__':
    rospy.init_node('put_object_on_surface_server')
    server = PutObjectOnSurfaceAction(rospy.get_name())
    rospy.spin()
