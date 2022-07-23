#! /usr/bin/env python3
""" Development action server for moving hand to a given tf frame.
Not used now.
"""

import rospy
import hsrb_interface.geometry as geometry

import orion_actions.msg as msg
from manipulation.collision_mapping import CollisionWorld
from manipulation.manipulation_header import ManipulationAction

# Enable robot interface
from hsrb_interface import robot as _robot

_robot.enable_interactive()


class MoveHandToTfAction(ManipulationAction):
    # create messages that are used to publish feedback/result
    _feedback = msg.MoveHandToTfActionFeedback()
    _result = msg.MoveHandToTfActionResult()

    # Whether to finally return to the map position the manipulation action was called at
    RETURN_TO_START_AFTER_ACTION = False

    def __init__(
        self,
        action_name,
        action_msg_type=msg.MoveHandToTfAction,
        use_collision_map=True,
        tts_narrate=True,
        prevent_motion=False,
    ):

        super(MoveHandToTfAction, self).__init__(
            action_name,
            action_msg_type,
            use_collision_map,
            tts_narrate,
            prevent_motion,
        )

        rospy.loginfo("%s: Initialised. Ready for clients." % self._action_name)

    def _execute_cb(self, goal_msg):
        """
        Action server callback for MoveHandToTfAction
        """
        goal_tf = goal_msg.goal_tf
        rospy.loginfo("%s: Requested to move hand to tf %s" % (self._action_name, goal_tf))

        # Attempt to find transform from hand frame to goal_tf
        (trans, lookup_time) = self.lookup_transform(self.HAND_FRAME, goal_tf)

        if trans is None:
            rospy.logerr("Unable to find TF frame")
            self.tts_say("I don't know that transform.", duration=2.0)
            self.abandon_action()
            return

        # Look at the object - make sure that we get all of the necessary collision map
        rospy.loginfo("%s: Moving head to look at the target." % self._action_name)
        self.look_at_object(goal_tf)

        if self.handle_possible_preemption():
            return

        # Evaluate collision environment
        if self.use_collision_map:
            self.tts_say("Evaluating a collision-free path.", duration=1.0)
            collision_world = self.get_goal_cropped_collision_map(goal_tf)
        else:
            collision_world = CollisionWorld.empty(self.whole_body)

        if self.handle_possible_preemption():
            return

        move_success = False
        try:
            with collision_world:
                self.whole_body.move_end_effector_pose(geometry.pose(), goal_tf)
            move_success = True

        except Exception as e:
            rospy.loginfo("%s: Encountered exception %s." % (self._action_name, str(e)))

        if move_success:
            rospy.loginfo("%s: Move succeeded" % self._action_name)
            self._result.result = True
            self._as.set_succeeded(self._result)
        else:
            rospy.loginfo("%s: Move failed" % self._action_name)
            self._result.result = False
            self._as.set_aborted()


if __name__ == "__main__":
    rospy.init_node("move_hand_to_tf_server_node")
    server = MoveHandToTfAction(
        "move_hand_to_tf", use_collision_map=True
    )
    rospy.spin()
