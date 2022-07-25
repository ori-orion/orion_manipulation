#! /usr/bin/env python3
""" Action server for pouring a held object "into" a TF.
        - Do not use with real milk.
"""

import rospy
import math
import hsrb_interface.geometry as geometry

import orion_actions.msg as msg
from manipulation.collision_mapping import CollisionWorld
from manipulation.manipulation_header import ManipulationAction

# Enable robot interface
from hsrb_interface import robot as _robot

_robot.enable_interactive()


class PourIntoAction(ManipulationAction):

    # Constants for pouring operation
    POUR_TIME = 4.0
    DEFAULT_POUR_HEIGHT = 0.05
    POUR_POSE = geometry.pose(x=0.1, z=-0.05)  # y=-0.20, z=-0.05, ek=-1.57

    def __init__(
        self,
        action_name,
        action_msg_type=msg.PourIntoAction,
        use_collision_map=True,
        tts_narrate=True,
        prevent_motion=False,
    ):

        super(PourIntoAction, self).__init__(
            action_name,
            action_msg_type,
            use_collision_map,
            tts_narrate,
            prevent_motion,
        )

        rospy.loginfo("%s: Initialised. Ready for clients." % self._action_name)

    def _execute_cb(self, goal_msg):
        """
        Action server callback for PourIntoAction
        """
        _result = msg.PourIntoResult()

        goal_tf = goal_msg.goal_tf
        rospy.loginfo("%s: Requested to pour into tf %s" % (self._action_name, goal_tf))

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
        self.tts_say("I will pour what I am holding into the specified object.")

        self.whole_body.move_to_neutral()

        try:
            with collision_world:
                # Move grasper over the object to pour into
                self.whole_body.move_end_effector_pose(self.POUR_POSE, goal_tf)

                # Attempt to start pour
                self.whole_body.move_end_effector_by_arc(
                    geometry.pose(x=self.DEFAULT_POUR_HEIGHT),
                    math.radians(90.0),
                    ref_frame_id="hand_palm_link",
                )

            # Delay to make sure pouring is complete
            rospy.sleep(self.POUR_TIME)
            move_success = True

        except Exception as e:
            rospy.loginfo("%s: Encountered exception %s." % (self._action_name, str(e)))

        if move_success:
            rospy.loginfo("%s: Pour succeeded" % self._action_name)
            _result.result = True
            self._as.set_succeeded(_result)
        else:
            rospy.loginfo("%s: Pour failed" % self._action_name)
            _result.result = False
            self._as.set_aborted()

        self.finish_position()

    def finish_position(self):
        rospy.loginfo("%s: Returning to go pose." % (self._action_name))
        self.whole_body.move_to_go()


if __name__ == "__main__":
    rospy.init_node("pour_into_server_node")
    server = PourIntoAction("pour_into")
    rospy.spin()
