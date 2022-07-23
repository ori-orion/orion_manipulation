#! /usr/bin/env python3
""" Action server for giving an object to operator.
Hard-coded hand over motion with a 5s pause for the operator to grasp
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


class GiveObjectToOperatorAction(ManipulationAction):
    def __init__(
        self,
        action_name,
        action_msg_type=msg.GiveObjectToOperatorAction,
        use_collision_map=False,
    ):

        super(GiveObjectToOperatorAction, self).__init__(
            action_name, action_msg_type, use_collision_map
        )
        rospy.loginfo("%s: Initialised. Ready for clients." % self._action_name)

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
            rospy.loginfo(
                "%s: Could not connect to speech. Giving operator object anyway..."
                % (self._action_name)
            )
            pass

    def _execute_cb(self, goal_msg):

        _result = msg.GiveObjectToOperatorResult()

        # Implemented a speech client but currently has no effect because it pass if fails
        # Just using this to test
        # self.get_speech_confirmation()

        try:
            rospy.loginfo(
                "%s: Moving to neutral position to present object."
                % (self._action_name)
            )
            self.whole_body.move_to_neutral()
            rospy.sleep(0.5)
            self.tts.say("I will now pass you the object.")
            rospy.sleep(1)

            rospy.loginfo("%s: Stretching out arm." % (self._action_name))
            self.whole_body.linear_weight = 100
            self.whole_body.move_end_effector_pose(
                geometry.pose(x=0.2, z=0.2), "hand_palm_link"
            )

            rospy.sleep(1)
            self.tts.say(
                "Please make sure you are holding the object and I will let go."
            )
            rospy.sleep(5)

            rospy.loginfo("%s: Opening gripper." % (self._action_name))
            self.gripper.command(1.2)
            rospy.loginfo("%s: Object given to operator." % (self._action_name))

            rospy.sleep(2)
            rospy.loginfo("%s: Moving to go position." % (self._action_name))

            _result.result = True
            self._as.set_succeeded(_result)

        except Exception as e:
            rospy.loginfo("%s: Exception encountered: %s." % (self._action_name, e))

            _result.result = False
            self._as.set_aborted()

        self.finish_position()

    def finish_position(self):
        self.whole_body.move_to_go()


if __name__ == "__main__":
    rospy.init_node("give_object_to_operator_server_node")
    server = GiveObjectToOperatorAction(
        "give_object_to_operator", use_collision_map=False
    )
    rospy.spin()
