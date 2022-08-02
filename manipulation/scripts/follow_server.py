#! /usr/bin/env python3
""" Action server for following an object (usually a person).
Takes a tf frame string as input to follow. Maintains 50cm distance away from object and follows
until pre-empted
"""

import rospy
import math
import hsrb_interface.geometry as geometry

import orion_actions.msg as msg
from manipulation.manipulation_header import (
    ManipulationAction,
    ROSServiceContextManager,
)
from std_srvs.srv import Empty, EmptyRequest
from geometry_msgs.msg import PoseStamped

# Enable robot interface
from hsrb_interface import robot as _robot

_robot.enable_interactive()


class FollowAction(ManipulationAction):

    # Whether to finally return to the map position the manipulation action was called at
    RETURN_TO_START_AFTER_ACTION = False
    RETURN_TO_START_GAZE_AFTER_ACTION = False

    # How recently the tf must have been published to be valid to follow
    FOLLOW_TF_TIMEOUT = 10.0
    MINIMUM_FOLLOW_DISTANCE = 1.5  # metres

    def __init__(
        self,
        action_name,
        action_msg_type=msg.FollowAction,
        use_collision_map=False,
        tts_narrate=True,
        prevent_motion=False,
    ):

        super(FollowAction, self).__init__(
            action_name,
            action_msg_type,
            use_collision_map,
            tts_narrate,
            prevent_motion,
        )

        self.pub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=10)

        rospy.loginfo("%s: Waiting for gaze control services..." % self._action_name)
        rospy.wait_for_service("viewpoint_controller/start")
        self.gaze_start_client = rospy.ServiceProxy(
            "/viewpoint_controller/start", Empty
        )
        rospy.wait_for_service("viewpoint_controller/stop")
        self.gaze_stop_client = rospy.ServiceProxy("/viewpoint_controller/stop", Empty)
        rospy.loginfo("%s: Got gaze control services" % self._action_name)

        # This context manager disables automatic gaze for the context under it
        self.disable_enable_gaze_context = ROSServiceContextManager(
            {self.gaze_stop_client: EmptyRequest()},
            {self.gaze_start_client: EmptyRequest()},
        )

        rospy.loginfo("%s: Initialised. Ready for clients." % self._action_name)

    def _execute_cb(self, goal_msg):
        _result = msg.FollowResult()
        _result.succeeded = False

        goal_tf = goal_msg.goal_tf
        rospy.loginfo("%s: Requested to follow tf %s" % (self._action_name, goal_tf))

        # Attempt to find transform from base footprint frame to goal_tf
        (trans, lookup_time) = self.lookup_transform(self.BASE_FOOTPRINT_FRAME, goal_tf)

        if trans is None:
            rospy.logerr("Unable to find TF frame")
            self.tts_say("I don't know that transform.", duration=2.0)
            self.abandon_action()
            return

        self.tts_say(
            "I will now start following. Please do not go too fast.", duration=2.0
        )

        while not rospy.is_shutdown():
            if self.handle_possible_preemption():
                return

            # Check the object is still in sight
            (trans, lookup_time) = self.lookup_transform(
                self.BASE_FOOTPRINT_FRAME, goal_tf
            )

            transform_age = (rospy.Time.now() - lookup_time).to_sec()
            if transform_age > self.FOLLOW_TF_TIMEOUT:
                rospy.logerr(
                    "%s: Most recent published target TF frame is too old (%.3g seconds old)."
                    % (self._action_name, transform_age)
                )
                self.tts_say("Lost track of target.", duration=2.0)
                self.abandon_action()
                return

            rospy.loginfo("%s: Moving head to look at target." % self._action_name)
            try:
                self.whole_body.gaze_point(ref_frame_id=goal_tf)
                rospy.sleep(1.0)
            except:
                rospy.loginfo("%s: Unable to point gaze at target." % self._action_name)

            if self.handle_possible_preemption():
                return

            distance_2d = self.calc_transform_distance(trans, only_2d=True)
            yaw = math.atan2(trans.translation.y, trans.translation.x)

            rospy.loginfo(
                "%s: Target distance: %.3gm, yaw %.3g rad."
                % (self._action_name, distance_2d, yaw)
            )

            # Prevent head from moving while navigating
            with self.disable_enable_gaze_context:

                if self.handle_possible_preemption():
                    return

                if distance_2d >= self.MINIMUM_FOLLOW_DISTANCE:
                    ratio = 1 - (0.5 / distance_2d)
                    rospy.loginfo("%s: Sending base goals." % self._action_name)
                    base_pose = geometry.pose(
                        ratio * trans.translation.x,
                        ratio * trans.translation.y,
                        0.0,
                        0.0,
                        0.0,
                        yaw,
                    )
                    base_goal = self.omni_base.create_go_pose_goal(
                        base_pose, "base_footprint"
                    )
                    self.omni_base.execute(base_goal)

                    rospy.loginfo(
                        "%s: Base movement complete. Continuing to follow."
                        % self._action_name
                    )


if __name__ == "__main__":
    rospy.init_node("follow_server_node")
    server = FollowAction("follow")
    rospy.spin()
