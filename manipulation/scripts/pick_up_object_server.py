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


import rospy
import actionlib
from actionlib_msgs.msg import GoalStatus
import hsrb_interface.geometry as geometry
from tmc_suction.msg import SuctionControlAction, SuctionControlGoal

from orion_actions.msg import *
from manipulation.manipulation_header import ManipulationAction
from point_cloud_filtering.srv import SegmentObject
from gpd.msg import GraspConfigList

# Enable robot interface
from hsrb_interface import robot as _robot

_robot.enable_interactive()


class PickUpObjectAction(ManipulationAction):

    ACTION_SERVER_CONNECTION_TIMEOUT = 15.0  # Used for e.g. vacuum action server
    GOAL_OBJECT_TF_TIMEOUT = (
        3.0  # How recently we must have seen an object to pick it up
    )

    SUCTION_TIMEOUT = rospy.Duration(20.0)  # Vacuum action timeout
    DEFAULT_GRASP_FORCE = 0.8
    PREGRASP_POSE = geometry.pose(z=-0.08, ek=0)  # Relative to gripper
    GRASP_POSE = geometry.pose(z=0.06)  # Relative to gripper

    def __init__(
        self,
        action_name,
        action_msg_type=orion_actions.msg.PickUpObjectAction,
        use_collision_map=True,
        use_grasp_synthesis=True,
    ):

        super(PickUpObjectAction, self).__init(
            action_name, action_msg_type, use_collision_map
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

    def segment_grasp_target_object(self, object_pos_head_frame):
        """
        Request the object_segmentation node to publish a segmented point cloud at a
        specified location in the head RGBD frame, containing the object we wish to
        generate grasp poses for.
        The grasp pose synthesis node will input this point cloud and publish a list of
        candidate poses.
        """
        try:
            response = segment_object_service(
                object_pos_head_frame[0],
                object_pos_head_frame[1],
                object_pos_head_frame[2],
            )
        except rospy.ServiceException as e:
            rospy.logerr("%s: Service call failed: %s" % (self._action_name, e))
            return False

        return response.result

    def get_hand_pose(self, goal_tf, relative=geometry.pose()):
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

    def _execute_cb(self, goal_msg):
        """
        Action server callback for PickUpObjectAction
        """
        _result = PickUpObjectResult()

        goal_tf = goal_msg.goal_tf
        rospy.loginfo("%s: Requested to pick up tf %s" % (self._action_name, goal_tf))

        # Attempt to find transform from hand frame to goal_tf
        (trans, lookup_time) = self.lookup_transform(self.HAND_FRAME, goal_tf)

        if trans is None:
            rospy.logerr("Unable to find TF frame")
            self.tts_say("I don't know the object you want picked up.")
            self.abandon_action()
            return

        # Look at the object - make sure that we get all of the necessary collision map
        rospy.loginfo("%s: Moving head to look at the object." % self._action_name)
        self.look_at_object(goal_tf)

        if (rospy.Time.now() - lookup_time).to_sec() > self.GOAL_OBJECT_TF_TIMEOUT:
            rospy.logerr("Most recent published goal TF frame is too old")
            self.tts_say("I can't see the object you want picked up.")
            self.abandon_action()
            return

        if self._as.is_preempt_requested():
            self.preempt_action()
            return

        obj_dist = self.calc_transform_distance(trans)
        rospy.loginfo(
            "%s: Distance to object is %s m from hand." % (self._action_name, obj_dist)
        )
        self.tts_say('The object is "{:.2f}" metres away.'.format(obj_dist))
        rospy.sleep(1)

        if self._as.is_preempt_requested():
            self.preempt_action()
            return

        # Evaluate collision environment
        if self.use_collision_map:
            self.tts_say(
                "I am now evaluating my environment so I don't collide with anything."
            )
            rospy.loginfo("%s: Getting Collision Map." % self._action_name)

            (trans, _) = self.lookup_transform(self.MAP_FRAME, goal_tf)
            goal_x = trans.translation.x
            goal_y = trans.translation.y
            goal_z = trans.translation.z

            external_bounding_box = BoundingBox(
                min=Point(goal_x - 0.7, goal_y - 0.7, goal_z - 1.0),
                max=Point(goal_x + 0.7, goal_y + 0.7, goal_z + 1.0),
            )

            # TODO this is a hard-coded ~10cm box
            object_bounding_box = BoundingBox(
                min=Point(goal_x - 0.05, goal_y - 0.05, goal_z - 0.03),
                max=Point(goal_x + 0.05, goal_y + 0.05, goal_z + 0.03),
            )

            # TODO this interface does not currently return collision world object
            self.collision_world = self.collision_mapper.build_collision_world(
                external_bounding_box, crop_bounding_boxes=[object_bounding_box]
            )

            rospy.loginfo("%s: Collision Map generated." % self._action_name)

        # Give opportunity to preempt
        if self._as.is_preempt_requested():
            self.preempt_action()
            return

        self.tts_say("I will now pick up the object")
        rospy.sleep(1)
        grab_success = self.grab_object(goal_tf, self.PREGRASP_POSE, self.GRASP_POSE)

        if not grab_success:
            self.tts_say("Failed to pick up the object")
        else:
            self.tts_say("Object grasped successfully.")
        rospy.sleep(1)

        # Give opportunity to preempt
        if self._as.is_preempt_requested():
            self.preempt_action()
            return

        # Now return to moving position
        # TODO consider making this return to a saved previous position
        self.finish_position()

        if grab_success:
            rospy.loginfo("%s: Succeeded" % self._action_name)
            _result.result = True
            self._as.set_succeeded(_result)
        else:
            rospy.loginfo("%s: Failed" % self._action_name)
            _result.result = False
            self._as.set_aborted()

    def grab_object(self, goal_tf, chosen_pregrasp_pose, chosen_grasp_pose):
        self.whole_body.end_effector_frame = self.HAND_FRAME

        rospy.loginfo("%s: Opening gripper." % (self._action_name))
        self.gripper.command(1.2)
        if self.use_collision_map:
            self.whole_body.collision_world = self.collision_world
        else:
            self.whole_body.collision_world = None

        try:
            if self.use_grasp_synthesis:
                # Segment the object point cloud first
                object_position_head_frame = self.get_head_frame_object_pose(goal_tf)

                # Call segmentation (lasts 10s)
                self.segment_object(object_position_head_frame)

                self.tts_say(
                    "I am trying to calculate the best possible grasp position"
                )
                rospy.sleep(1)
                # Get the best grasp - returns the pose-tuple in the head-frame
                grasp = self.get_grasp()

                rospy.loginfo("%s: Moving to pre-grasp position." % (self._action_name))
                self.tts_say("I will now move into the grasp position")
                rospy.sleep(1)
                self.whole_body.move_end_effector_pose(
                    grasp, "head_rgbd_sensor_rgb_frame"
                )

            else:
                # Move to pregrasp
                rospy.loginfo("%s: Calculating grasp pose." % (self._action_name))

                hand_pose = self.get_hand_pose(goal_tf, relative=chosen_pregrasp_pose)

                # Error checking in case can't find goal pose
                if hand_pose is None:
                    self.abandon_action()
                    return False

                self.publish_goal_pose_tf(hand_pose)

                rospy.loginfo("%s: Moving to pre-grasp position." % (self._action_name))
                self.tts_say("Moving to pre-grasp position.")
                self.whole_body.move_end_effector_pose(hand_pose, "odom")

            # Turn off collision checking to get close and grasp
            rospy.loginfo(
                "%s: Turning off collision checking to get closer."
                % (self._action_name)
            )
            self.whole_body.collision_world = None
            rospy.sleep(1)

            # Move to grasp pose
            rospy.loginfo("%s: Moving to grasp position." % (self._action_name))
            self.tts_say("Moving to grasp position.")
            rospy.sleep(1)
            self.whole_body.move_end_effector_pose(
                chosen_grasp_pose, self.whole_body.end_effector_frame
            )

            # Specify the force to grasp
            self.tts_say("Grasping object.")
            rospy.sleep(1)
            self.gripper.apply_force(self._GRASP_FORCE)
            rospy.loginfo("%s: Object grasped." % self._action_name)

            return True

        except Exception as e:
            rospy.loginfo("%s: Encountered exception %s." % (self._action_name, str(e)))
            self.whole_body.collision_world = None
            self.abandon_action()
            return False

    def suck_object(self, goal_tf):
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

        if self.use_collision_map:
            self.whole_body.collision_world = self.collision_world
        else:
            self.whole_body.collision_world = None
        self.whole_body.move_end_effector_pose(geometry.pose(z=0.05, ei=3.14), goal_tf)

        # Turn off collision checking to get close and grasp
        rospy.loginfo(
            "%s: Turning off collision checking to get closer." % (self._action_name)
        )
        self.whole_body.collision_world = None
        self.whole_body.move_end_effector_pose(geometry.pose(z=0.045), goal_tf)

    def finish_position(self):
        if self.use_collision_map:
            self.whole_body.collision_world = self.collision_world
        else:
            self.whole_body.collision_world = None

        try:
            rospy.loginfo(
                "%s: Trying to move back and get into go position." % self._action_name
            )
            self.omni_base.go_rel(-0.3, 0, 0)
            self.whole_body.move_to_go()
            return True
        except Exception as e:
            rospy.loginfo("%s: Encountered exception %s." % (self._action_name, str(e)))
            self.whole_body.collision_world = None
            try:
                rospy.loginfo(
                    "%s: Moving back and attempting to move to go again without collision detection."
                    % self._action_name
                )

                try:
                    self.omni_base.go_rel(-0.3, 0, 0)
                except:
                    rospy.loginfo(
                        "%s: Trying to move to the side instead." % self._action_name
                    )
                    try:
                        self.omni_base.go_rel(0, 0.3, 0)
                    except:
                        self.omni_base.go_rel(0, -0.3, 0)
                    self.whole_body.move_to_go()
            except:
                self.omni_base.go_rel(-0.3, 0, 0)
                self.whole_body.move_to_go()
            return False

    def grasp_callback(self, msg):
        self.grasps = msg.grasps

    def get_grasp(self):
        """
        For grasp pose synthesis.
        """

        while not len(self.grasps) > 0:
            if len(self.grasps) > 0:
                rospy.loginfo("Received %d grasps.", len(self.grasps))
                break

        grasp = self.grasps[0]  # grasps are sorted in descending order by score
        rospy.loginfo(
            "%s: Selected grasp with score:: %s" % (self._action_name, str(grasp.score))
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


if __name__ == "__main__":
    rospy.init_node("pick_up_object_server_node")
    server = PickUpObjectAction(rospy.get_name())
    rospy.spin()
