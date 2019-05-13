#! /usr/bin/env python

import hsrb_interface
import rospy
import actionlib
import hsrb_interface.geometry as geometry

from actionlib_msgs.msg import GoalStatus

from orion_actions.msg import ReceiveObjectFromOperatorAction, ReceiveObjectFromOperatorGoal, ReceiveObjectFromOperatorAction
from orion_hri.msg import WaitForConfirmationAction, WaitForConfirmationGoal, WaitForConfirmationResult

class ReceiveObjectFromOperatorAction(object):

    def __init__(self, name):
        self._action_name = 'receive_object_from_operator'
        self._as = actionlib.SimpleActionServer(self._action_name, orion_actions.msg.ReceiveObjectFromOperatorAction,execute_cb=self.execute_cb, auto_start=False)
        self._as.start()

        # Preparation for using the robot functions
        self.robot = hsrb_interface.Robot()
        self.whole_body = self.robot.try_get('whole_body')
        self.gripper = self.robot.try_get('gripper')

        # Increase planning timeout. Default is 10s
        self.whole_body.planning_timeout = 20.0
        self._CONNECTION_TIMEOUT = 10

        # Specify the force to grasp
        self._GRASP_FORCE = 0.8

        rospy.loginfo('%s: Initialised. Ready for clients.' % (self._action_name))


    def execute_cb(self, goal_msg):

        _result = ReceiveObjectFromOperatorAction()

        try:
            rospy.loginfo('%s: Moving to neutral position to present object.' % (self._action_name))
            self.whole_body.move_to_go()

            rospy.loginfo('%s: Stretching out arm.' % (self._action_name))
            self.whole_body.move_end_effector_pose(geometry.pose(z=0.1), 'hand_palm_link')

            rospy.sleep(7)

            rospy.loginfo('%s: Closing gripper.' % (self._action_name))
            self.gripper.apply_force(self._GRASP_FORCE)
            rospy.loginfo('%s: Object grasped.' % self._action_name)

            self.whole_body.move_to_neutral()

            _result.result = True
            self._as.set_succeeded(_result)
        except Exception as e:
            rospy.loginfo('%s: Exception encountered: %s.' % (self._action_name, e))
            self._as.set_aborted()


if __name__ == '__main__':
    rospy.init_node('give_object_to_operator_server_node')
    server = ReceiveObjectFromOperatorAction(rospy.get_name())
    rospy.spin()
