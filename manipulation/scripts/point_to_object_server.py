#! /usr/bin/env python3
""" Action server for pointing objects.
Takes a tf frame string and the robot will turn and point towards the object.
"""

import rospy
import hsrb_interface.geometry as geometry

import orion_actions.msg as msg
from manipulation.manipulation_header import ManipulationAction

# Enable robot interface
from hsrb_interface import robot as _robot

_robot.enable_interactive()


class PointToObjectAction(ManipulationAction):

    # Whether to finally return to the map position the manipulation action was called at
    RETURN_TO_START_AFTER_ACTION = False

    def __init__(
        self,
        action_name,
        action_msg_type=msg.PointToObjectAction,
        use_collision_map=False,
        tts_narrate=True,
        prevent_motion=False,
    ):

        super(PointToObjectAction, self).__init__(
            action_name,
            action_msg_type,
            use_collision_map,
            tts_narrate,
            prevent_motion,
        )

        rospy.loginfo("%s: Initialised. Ready for clients." % self._action_name)

    def _execute_cb(self, goal_msg):
        """
        Action server callback for PointToObjectAction
        """
        _result = msg.PointToObjectResult()

        goal_tf = goal_msg.goal_tf
        rospy.loginfo("%s: Requested to move hand to tf %s" % (self._action_name, goal_tf))

        rospy.loginfo("%s: Closing gripper to point at object" % self._action_name)
        self.gripper.set_distance(0.01)

        # Attempt to find transform from hand frame to goal_tf
        (trans, lookup_time) = self.lookup_transform(self.HAND_FRAME, goal_tf)

        if trans is None:
            rospy.logerr("Unable to find TF frame")
            self.tts_say("I don't know that transform.", duration=2.0)
            self.abandon_action()
            return

        # Look at the object
        rospy.loginfo("%s: Moving head & body to look at the target." % self._action_name)
        self.look_at_object(goal_tf, rotate_to_face=True)

        if self.handle_possible_preemption():
            return

        # Update transform from hand frame to goal_tf
        (trans, lookup_time) = self.lookup_transform(self.HAND_FRAME, goal_tf)

        obj_dist = self.calc_transform_distance(trans)
        if obj_dist > 0.3:
            req_ratio = 0.3 / obj_dist
        else:
            req_ratio = 0.5

        self.tts_say("Pointing to object", duration=1.0)

        if self.handle_possible_preemption():
            return

        rospy.loginfo("%s: Pointing to object." % self._action_name)
        self.whole_body.looking_hand_constraint = False

        self.whole_body.gaze_point(
            point=geometry.Vector3(0, 0, 0), ref_frame_id=goal_tf
        )

        self.whole_body.linear_weight = 100
        self.whole_body.move_end_effector_pose(
            geometry.pose(
                x=req_ratio * trans.translation.x,
                y=req_ratio * trans.translation.y,
                z=req_ratio * trans.translation.z,
            ),
            self.HAND_FRAME,
        )

        self.tts_say("It is over there.", duration=1.0)
        rospy.sleep(3)
        self.finish_position()

        self.whole_body.looking_hand_constraint = True
        rospy.loginfo("%s: Succeeded" % self._action_name)
        _result.result = True
        self._as.set_succeeded(_result)

    def finish_position(self):
        rospy.loginfo("%s: Returning to go pose." % (self._action_name))
        self.whole_body.move_to_go()


if __name__ == "__main__":
    rospy.init_node("point_to_object_server_node")
    server = PointToObjectAction("point_to_object")
    rospy.spin()
