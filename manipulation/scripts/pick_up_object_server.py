#! /usr/bin/env python3
""" Action server for picking up objects.
Key options:
    - use_grasp_synthesis:
        True:   Use gpd / gpg NN-based grasp pose synthesis.
        False:  Use hard-coded grasp synthesis (a few cm from target centre, horizontal).
    - use_collision_map:
        True:   Automatically avoid obstacles.
        False:  Do not avoid obstacles. ONLY FOR SUPERVISED TESTING WITH E-STOP.
"""

import numpy as np
import rospy
import actionlib
import tf
import tf.transformations as T
import traceback
import hsrb_interface.geometry as geometry

import orion_actions.msg as msg
from actionlib_msgs.msg import GoalStatus
from manipulation.manipulation_header import ManipulationAction
from manipulation.collision_mapping import CollisionWorld
from point_cloud_filtering.srv import SegmentObject
from gpd.msg import GraspConfigList
from tmc_suction.msg import SuctionControlAction, SuctionControlGoal

# Enable robot interface
from hsrb_interface import robot as _robot

_robot.enable_interactive()


class PickUpObjectAction(ManipulationAction):

    ACTION_SERVER_CONNECTION_TIMEOUT = 15.0  # Used for e.g. vacuum action server
    GOAL_OBJECT_TF_TIMEOUT = (
        100.0  # How recently we must have seen an object to pick it up
    )
    GRASP_POSE_GENERATION_TIMEOUT = (
        7.0  # How long we give grasp pose generation to generate grasps
    )

    SUCTION_TIMEOUT = rospy.Duration(20.0)  # Vacuum action timeout
    DEFAULT_GRASP_FORCE = 0.8
    PREGRASP_POSE = geometry.pose(z=-0.08, ek=0)  # Relative to gripper
    GRASP_POSE = geometry.pose(z=0.06)  # Relative to gripper

    def __init__(
        self,
        action_name,
        action_msg_type=msg.PickUpObjectAction,
        use_collision_map=True,
        use_grasp_synthesis=True,
        tts_narrate=True,
        prevent_motion=False,
    ):

        super(PickUpObjectAction, self).__init__(
            action_name,
            action_msg_type,
            use_collision_map,
            tts_narrate,
            prevent_motion,
        )

        self.use_grasp_synthesis = use_grasp_synthesis

        if self.use_grasp_synthesis:
            rospy.loginfo("%s: Grasp pose synthesis is enabled" % self._action_name)
            rospy.loginfo(
                "%s: Waiting for object_segmentation service..." % self._action_name
            )
            rospy.wait_for_service("/object_segmentation")
            self.segment_object_service = rospy.ServiceProxy(
                "/object_segmentation", SegmentObject
            )
            rospy.loginfo("%s: Got object_segmentation service" % self._action_name)

            self.grasps = None
            self.grasp_sub = rospy.Subscriber(
                "/detect_grasps/clustered_grasps", GraspConfigList, self.grasp_callback
            )
        rospy.loginfo("%s: Initialised. Ready for clients." % self._action_name)

    def _execute_cb(self, goal_msg):
        """
        Action server callback for PickUpObjectAction
        """
        _result = msg.PickUpObjectResult()

        goal_tf = goal_msg.goal_tf
        rospy.loginfo("%s: Requested to pick up tf %s" % (self._action_name, goal_tf))

        # Attempt to find transform from hand frame to goal_tf
        (trans, lookup_time) = self.lookup_transform(self.HAND_FRAME, goal_tf)

        if trans is None:
            rospy.logerr("Unable to find TF frame")
            self.tts_say("I don't know the object you want picked up.", duration=2.0)
            self.abandon_action()
            return

        # Look at the object - make sure that we get all of the necessary collision map
        rospy.loginfo("%s: Moving head to look at the object." % self._action_name)
        self.look_at_object(goal_tf)

        if self.handle_possible_preemption():
            return

        if (rospy.Time.now() - lookup_time).to_sec() > self.GOAL_OBJECT_TF_TIMEOUT:
            rospy.logerr(
                "Most recent published goal TF frame is too old ({} seconds old)".format(
                    (rospy.Time.now() - lookup_time).to_sec()
                )
            )
            self.tts_say("I can't see the object you want picked up.", duration=2.0)
            self.abandon_action()
            return

        if self.handle_possible_preemption():
            return

        obj_dist = self.calc_transform_distance(trans)
        rospy.loginfo(
            "%s: Distance to object is %s m from hand." % (self._action_name, obj_dist)
        )
        self.tts_say(
            'The object is "{:.2f}" metres away.'.format(obj_dist), duration=2.0
        )

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

        grasp_success = self.grab_object(
            goal_tf, collision_world, self.PREGRASP_POSE, self.GRASP_POSE
        )

        if self.handle_possible_preemption():
            return

        if grasp_success:
            self.tts_say("Object grasped successfully.")
            # Now return to moving position
            self.finish_position(collision_world)
        else:
            self.tts_say("Failed to pick up the object")

        if grasp_success:
            rospy.loginfo("%s: Grasping succeeded" % self._action_name)
            _result.result = True
            self._as.set_succeeded(_result)
        else:
            rospy.loginfo("%s: Grasping failed" % self._action_name)
            _result.result = False
            self._as.set_aborted()

    def grab_object(
        self, goal_tf, collision_world, chosen_pregrasp_pose, chosen_grasp_pose
    ):
        self.whole_body.end_effector_frame = self.HAND_FRAME

        rospy.loginfo("%s: Opening gripper." % (self._action_name))
        self.gripper.command(1.2)

        rospy.loginfo("Is using collision mapping: %s" % (self.use_collision_map))
        rospy.loginfo("Is using grasp synthesis: %s" % (self.use_grasp_synthesis))

        try:
            successfully_positioned = False

            if self.use_grasp_synthesis:
                successfully_positioned = (
                    self.position_end_effector_grasp_pose_synthesis(
                        goal_tf, collision_world, chosen_grasp_pose
                    )
                )

            if not successfully_positioned:
                successfully_positioned = self.position_end_effector_fixed_grasp(
                    goal_tf, collision_world, chosen_pregrasp_pose, chosen_grasp_pose
                )

            if not successfully_positioned:
                return False

            # Specify the force to grasp
            self.tts_say("Grasping object.")
            self.gripper.apply_force(self.DEFAULT_GRASP_FORCE)
            rospy.loginfo("%s: Object grasped." % self._action_name)

            return True

        except Exception as e:
            rospy.logerr("%s: Encountered exception %s." % (self._action_name, str(e)))
            rospy.logerr(traceback.format_exc())
            self.abandon_action()
            return False

    def position_end_effector_grasp_pose_synthesis(
        self, goal_tf, collision_world, chosen_grasp_pose
    ):
        """
        Move to a valid grasping position using grasp pose synthesis.
        """
        # Clear the previous grasp list
        self.grasps = None

        # Segment the object point cloud first
        (head_obj_transform, _) = self.lookup_transform(self.RGBD_CAMERA_FRAME, goal_tf)

        rospy.loginfo("%s: Calling segmentation." % (self._action_name))
        # Call segmentation (lasts 10s)
        self.tts_say("I am trying to calculate the best possible grasp position")
        seg_response = self.segment_grasp_target_object(
            [
                head_obj_transform.translation.x,
                head_obj_transform.translation.y,
                head_obj_transform.translation.z,
            ]
        )

        rospy.loginfo("%s: Finished calling segmentation." % (self._action_name))

        if not seg_response:
            # Segmentation internal failure
            return False

        rospy.loginfo("%s: Waiting for grasps." % (self._action_name))
        # Grasps are sorted in descending order by score
        # Include a timeout
        start_time = rospy.Time.now()
        while self.grasps is None:
            rospy.sleep(0.2)
            elapsed_secs = (rospy.Time.now() - start_time).to_sec()
            if elapsed_secs > self.GRASP_POSE_GENERATION_TIMEOUT:
                break

        rospy.loginfo(
            "%s: Got grasps list of length %i." % (self._action_name, len(self.grasps))
        )

        if self.grasps is None:
            # gpg failed to return a list of grasp poses
            rospy.loginfo(
                "%s: GPG failed to produce grasp poses." % (self._action_name)
            )
            return False

        with collision_world:
            found_valid_grasp = False
            for idx, grasp in enumerate(self.grasps):
                rospy.loginfo("Grasping trial %s" % idx)
                grasp_pose = self.grasp_to_pose(self.grasps[idx])
                self.publish_goal_pose_tf(grasp_pose)

                try:
                    self.whole_body.move_end_effector_pose(
                        grasp_pose, self.RGBD_CAMERA_FRAME
                    )
                    found_valid_grasp = True
                except:
                    # Failed to find valid motion to this grasp pose - continue search
                    pass

        if not found_valid_grasp:
            return False

        # Robot should now have successfully moved to target generated grasp pose,
        # taking offset into account, and using collision avoidance if necessary.

        # Move to grasp pose without collision checking
        rospy.loginfo("%s: Moving to grasp." % (self._action_name))
        self.tts_say("Moving to grasp.")
        self.whole_body.move_end_effector_pose(
            chosen_grasp_pose, self.whole_body.end_effector_frame
        )
        rospy.loginfo("%s: Finished moving to grasp position." % (self._action_name))
        return True

    def position_end_effector_fixed_grasp(
        self, goal_tf, collision_world, chosen_pregrasp_pose, chosen_grasp_pose
    ):
        """
        Move to a valid grasping position using a fixed approach from robot to target.
        """

        # Move to pregrasp
        rospy.loginfo("%s: Calculating grasp pose." % (self._action_name))

        hand_pose = self.get_default_grasp_pose(goal_tf, relative=chosen_pregrasp_pose)

        # Error checking in case can't find goal pose
        if hand_pose is None:
            self.abandon_action()
            return False

        self.publish_goal_pose_tf(hand_pose)

        rospy.loginfo("%s: Moving to pre-grasp position." % (self._action_name))
        self.tts_say("Moving to pre-grasp.")

        with collision_world:
            self.whole_body.move_end_effector_pose(hand_pose, "odom")

        # Move to grasp pose without collision checking
        rospy.loginfo("%s: Moving to grasp." % (self._action_name))
        self.tts_say("Moving to grasp.")
        self.whole_body.move_end_effector_pose(
            chosen_grasp_pose, self.whole_body.end_effector_frame
        )
        return True

    def suck_object(self, goal_tf, collision_world):
        """
        Currently unused.
        """
        self.whole_body.end_effector_frame = self.SUCKER_FRAME

        rospy.loginfo("%s: Closing gripper." % self._action_name)
        self.gripper.command(0.1)

        rospy.loginfo("%s: Turning on on the suction..." % self._action_name)

        # Create action client to control suction
        suction_action = "/hsrb/suction_control"
        suction_control_client = actionlib.SimpleActionClient(
            suction_action, SuctionControlAction
        )

        # Wait for connection
        try:
            if not suction_control_client.wait_for_server(
                rospy.Duration(self.ACTION_SERVER_CONNECTION_TIMEOUT)
            ):
                raise Exception(suction_action + " does not exist")
        except Exception as e:
            rospy.logerr(e)

        # Send a goal to start suction
        rospy.loginfo(
            "%s: Suction server found. Activating suction..." % self._action_name
        )
        suction_on_goal = SuctionControlGoal()
        suction_on_goal.timeout = self._SUCTION_TIMEOUT
        suction_on_goal.suction_on.data = True

        if (
            suction_control_client.send_goal_and_wait(suction_on_goal)
            == GoalStatus.SUCCEEDED
        ):
            rospy.loginfo("Suction succeeded. Object picked up")
        else:
            rospy.loginfo("Suction failed")
            return

        with collision_world:
            self.whole_body.move_end_effector_pose(
                geometry.pose(z=0.05, ei=3.14), goal_tf
            )

        # Get close and grasp without collision checking
        self.whole_body.move_end_effector_pose(geometry.pose(z=0.045), goal_tf)

    def finish_position(self, collision_world):
        """
        Try to revert the robot to go position, free of any obstacles.
        """

        try:
            rospy.loginfo(
                "%s: Trying to move back and get into go position." % self._action_name
            )
            with collision_world:
                self.omni_base.go_rel(-0.3, 0, 0)
                self.whole_body.move_to_go()
            return True

        except Exception as e:
            rospy.loginfo("%s: Encountered exception %s." % (self._action_name, str(e)))

        # Unsuccessful - try without collision avoidance
        try:
            rospy.loginfo(
                "%s: Retrying without collision detection." % self._action_name
            )
            self.omni_base.go_rel(-0.3, 0, 0)
            self.whole_body.move_to_go()
            return True

        except Exception as e:
            rospy.loginfo("%s: Encountered exception %s." % (self._action_name, str(e)))

        # Unsuccessful - try going sideways
        try:
            rospy.loginfo("%s: Trying to move to the side instead." % self._action_name)
            self.omni_base.go_rel(0, 0.3, 0)
            self.whole_body.move_to_go()
            return True

        except Exception as e:
            rospy.loginfo("%s: Encountered exception %s." % (self._action_name, str(e)))

        return False

    def get_default_grasp_pose(self, goal_tf, relative=geometry.pose()):
        """
        Get a hsrb_interface.geometry Pose tuple representing a relative pose from the
        goal tf, all in frame "odom".
        Args:
            goal_tf: goal tf
            relative: relative hsrb_interface.geometry pose to goal tf to get hand pose
        """

        (trans, lookup_time) = self.lookup_transform(self.ODOM_FRAME, goal_tf)

        odom_to_ref = geometry.transform_to_tuples(trans)
        odom_to_hand = geometry.multiply_tuples(odom_to_ref, relative)

        return odom_to_hand

    def grasp_callback(self, msg):
        self.grasps = msg.grasps

    def grasp_to_pose(self, grasp):
        """
        Convert a grasp pose synthesis-generated grasp into a pose for the actuator.
        """
        rospy.loginfo(
            "%s: Evaluating grasp with score:: %s"
            % (self._action_name, str(grasp.score))
        )

        # This gives the approach point correctly
        bottom = np.array([grasp.bottom.x, grasp.bottom.y, grasp.bottom.z])
        approach = np.array([grasp.approach.x, grasp.approach.y, grasp.approach.z])
        binormal = np.array([grasp.binormal.x, grasp.binormal.y, grasp.binormal.z])
        hand_outer_diameter = 0.12
        hw = 0.5 * hand_outer_diameter
        finger_width = 0.01
        left_bottom = bottom - (hw - 0.5 * finger_width) * binormal
        right_bottom = bottom + (hw - 0.5 * finger_width) * binormal
        base_center = left_bottom + 0.5 * (right_bottom - left_bottom) - 0.01 * approach
        approach_center = base_center - 0.06 * approach

        approach_4 = np.array(
            [grasp.approach.x, grasp.approach.y, grasp.approach.z, approach_center[0]]
        )
        binormal_4 = np.array(
            [grasp.binormal.x, grasp.binormal.y, grasp.binormal.z, approach_center[1]]
        )
        axis_4 = np.array(
            [grasp.axis.x, grasp.axis.y, grasp.axis.z, approach_center[2]]
        )

        R = np.array([axis_4, -binormal_4, approach_4, [0, 0, 0, 1]])
        q = T.quaternion_conjugate(T.quaternion_from_matrix(R))

        return geometry.Pose(
            geometry.Vector3(
                approach_center[0], approach_center[1], approach_center[2]
            ),
            geometry.Quaternion(q[0], q[1], q[2], q[3]),
        )

    def segment_grasp_target_object(self, object_pos_head_frame):
        """
        Request the object_segmentation node to publish a segmented point cloud at a
        specified location in the head RGBD frame, containing the object we wish to
        generate grasp poses for.
        The grasp pose synthesis node will input this point cloud and publish a list of
        candidate poses.
        """
        try:
            self.segment_object_service(
                object_pos_head_frame[0],
                object_pos_head_frame[1],
                object_pos_head_frame[2],
            )
            return True
        except rospy.ServiceException as e:
            rospy.logerr("%s: Service call failed: %s" % (self._action_name, e))
            return False


if __name__ == "__main__":
    rospy.init_node("pick_up_object_server_node")
    server = PickUpObjectAction(
        "pick_up_object", use_collision_map=True, use_grasp_synthesis=True
    )
    rospy.spin()
