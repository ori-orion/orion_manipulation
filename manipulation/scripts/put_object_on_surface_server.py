#! /usr/bin/env python3
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
from manipulation.manipulation_header import ManipulationAction
from orion_actions.msg import *


class PutObjectOnSurfaceAction(ManipulationAction):

    def __init__(self, action_name, action_msg_type=orion_actions.msg.PutObjectOnSurfaceAction, use_collision_map=True):
               
        super(PutObjectOnSurfaceAction, self).__init__(
            action_name, action_msg_type, use_collision_map
        )

        rospy.loginfo('%s: Initialised. Ready for clients.' % self._action_name)

    def _execute_cb(self, goal_msg):
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
            self.whole_body.move_end_effector_pose(geometry.pose(x=0, y=0, z=0.2), 'hand_palm_link')

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
    server = PutObjectOnSurfaceAction('put_object_on_surface', use_collision_map=False)
    rospy.spin()
