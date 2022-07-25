#! /usr/bin/env python3
""" Action server for receiving objects.
Hard-coded motion to put hand out in front, pause and grasp an object.
"""

import rospy
import actionlib
import hsrb_interface.geometry as geometry

import orion_actions.msg as msg
from actionlib_msgs.msg import GoalStatus
from manipulation.manipulation_header import ManipulationAction
from orion_hri.msg import (
    WaitForConfirmationAction,
    WaitForConfirmationGoal,
    WaitForConfirmationResult,
)

# Enable robot interface
from hsrb_interface import robot as _robot

_robot.enable_interactive()


class ReceiveObjectFromOperatorAction(ManipulationAction):

    DEFAULT_GRASP_FORCE = 0.8

    def __init__(
        self,
        action_name,
        action_msg_type=msg.ReceiveObjectFromOperatorAction,
        use_collision_map=False,
    ):

        super(ReceiveObjectFromOperatorAction, self).__init__(
            action_name, action_msg_type, use_collision_map
        )
        rospy.loginfo("%s: Initialised. Ready for clients." % self._action_name)

    def _execute_cb(self, goal_msg):
        """
        Action server callback for PickUpObjectAction
        """
        _result = msg.ReceiveObjectFromOperatorResult()

        # NOTE not tested therefore disabled
        # self.get_speech_confirmation()

        try:
            rospy.loginfo("%s: Opening gripper." % (self._action_name))
            self.gripper.set_distance(1)

            rospy.loginfo(
                "%s: Moving to neutral position to receive object."
                % (self._action_name)
            )
            self.whole_body.move_to_neutral()

            rospy.loginfo("%s: Stretching out arm." % (self._action_name))
            self.tts_say("Moving into position to receive object.", duration=1.0)
            self.whole_body.linear_weight = 100
            self.whole_body.move_end_effector_pose(
                geometry.pose(x=0.2, z=0.2), "hand_palm_link"
            )

            rospy.sleep(1)
            rospy.loginfo("%s: Waiting for object." % (self._action_name))
            self.tts_say("Please place in the object in my gripper and I will take it.")
            rospy.sleep(8)

            rospy.loginfo("%s: Closing gripper." % (self._action_name))
            self.gripper.apply_force(self.DEFAULT_GRASP_FORCE)
            rospy.loginfo("%s: Object grasped." % self._action_name)

            self.tts_say("Thank you.", duration=1.0)

            _result.result = True
            self._as.set_succeeded(_result)

        except Exception as e:
            rospy.loginfo("%s: Exception encountered: %s." % (self._action_name, e))

            _result.result = False
            self._as.set_aborted()

        self.finish_position()

    def finish_position(self):
        rospy.loginfo("%s: Returning to go pose." % (self._action_name))
        self.tts_say("Returning to go position.")
        self.whole_body.move_to_go()

    def get_speech_confirmation(self):
        # Create action client to speech confirmation
        speech_confirm_action = "wait_for_confirmation"
        speech_confirm_client = actionlib.SimpleActionClient(
            speech_confirm_action, WaitForConfirmationAction
        )
        rospy.loginfo(" Connecting to speech confirmation server...")

        try:
            if not speech_confirm_client.wait_for_server(
                rospy.Duration(self._CONNECTION_TIMEOUT)
            ):
                raise Exception(speech_confirm_action + " does not exist")

            # Send a goal to start speech
            rospy.loginfo("Speech server found. Sending goal...")
            speech_goal = WaitForConfirmationGoal()
            speech_goal.question = "Operator, are you there?"
            speech_goal.possible_inputs = [
                rospy.getparam("bring_me/confirm_operator_presence"),
                rospy.getparam("bring_me/negative_confirmation"),
            ]
            # Note this timeout is currently
            speech_goal.timeout = rospy.getparam(
                "bring_me/wait_for_confirmation_timeout"
            )

            speech_confirm_client.send_goal(speech_goal)

            if (
                speech_confirm_client.send_goal_and_wait(
                    speech_goal, preempt_timeout=rospy.Duration(20)
                )
                == GoalStatus.SUCCEEDED
            ):
                rospy.loginfo("Speech succeeded.")
            else:
                rospy.loginfo("Speech failed.")

        except Exception as e:
            rospy.loginfo("%s: Encountered exception: %s" % (self._action_name, e))
            pass


if __name__ == "__main__":
    rospy.init_node("receive_object_from_operator_server_node")
    server = ReceiveObjectFromOperatorAction(
        "receive_object_from_operator", use_collision_map=False
    )
    rospy.spin()
