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
from geometry_msgs.msg import Point
from manipulation.msg import BoundingBox

import numpy as np
import tf
import tf.transformations as T
import math

from orion_actions.msg import *
from manipulation.manipulation_header import ManipulationAction
from point_cloud_filtering.srv import SegmentObject
from gpd.msg import GraspConfigList

# Enable robot interface
from hsrb_interface import robot as _robot

_robot.enable_interactive()

# How many times can we fail to find the Tf frame before returning?
NUM_TF_FAILS=30

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

        super(PickUpObjectAction, self).__init__(
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

            self.grasps = None
            self.goal_object = None
            grasp_sub = rospy.Subscriber('/detect_grasps/clustered_grasps', GraspConfigList, self.grasp_callback)


    def segment_grasp_target_object(self, object_pos_head_frame):
        """
        Request the object_segmentation node to publish a segmented point cloud at a
        specified location in the head RGBD frame, containing the object we wish to
        generate grasp poses for.
        The grasp pose synthesis node will input this point cloud and publish a list of
        candidate poses.
        """

# class PickUpObjectAction(object):

#     def __init__(self, name):
#         self._action_name = 'pick_up_object'
#         self._as = actionlib.SimpleActionServer(self._action_name, 	orion_actions.msg.PickUpObjectAction,
#                                                 execute_cb=self.execute_cb, auto_start=False)
#         self._as.start()
#         rospy.loginfo('%s: Action name is: %s' % (self._action_name, name))

#         self.use_collision_map = False

#         # Preparation for using the robot functions
#         self.robot = hsrb_interface.Robot()
#         self.whole_body = self.robot.try_get('whole_body')
#         self.omni_base = self.robot.try_get('omni_base')
#         self.collision_world = None
#         self.gripper = self.robot.try_get('gripper')
#         self.whole_body.end_effector_frame = 'hand_palm_link'
#         self.whole_body.looking_hand_constraint = True        
#         self.tts = self.robot.try_get('default_tts')
#         self.tts.language = self.tts.ENGLISH

#         if self.use_collision_map:
#             self.collision_mapper = CollisionMapper(self.robot)

#         self._CONNECTION_TIMEOUT = 15.0 # Define the vacuum timeouts
#         self._SUCTION_TIMEOUT = rospy.Duration(20.0) # Define the vacuum timeouts
#         self._HAND_TF = 'hand_palm_link'
#         self._GRASP_FORCE = 0.8
#         self.whole_body.planning_timeout = 20.0 # Increase planning timeout. Default is 10s
#         self.whole_body.tf_timeout = 10.0 # Increase tf timeout. Default is 5s

#         # Set up publisher for the collision map
#         self.pub = rospy.Publisher('known_object', CollisionObject, queue_size=1)
#         self.goal_pose_br = tf.TransformBroadcaster()

#         self.grasps = None
#         self.use_grasp_synthesis = True

#         self.goal_object = None
#         rospy.loginfo('%s: Initialised. Ready for clients.' % self._action_name)
#         grasp_sub = rospy.Subscriber('/detect_grasps/clustered_grasps', GraspConfigList, self.grasp_callback)
#         self.geting_grasps=False

#     def grasp_callback(self, msg):
#         # if self.getting_grasps:
#         self.grasps = msg.grasps


#     def get_grasp(self):
#         print('self.grasps:',self.grasps)
#         while not len(self.grasps) > 0:
#             if len(self.grasps) > 0:
#                 rospy.loginfo('Received %d grasps.', len(self.grasps))
#                 break

#         grasp = self.grasps[0] # grasps are sorted in descending order by score
#         rospy.loginfo('%s: Selected grasp with score:: %s' % (self._action_name, str(grasp.score)))

#         # This gives the approach point correctly
#         bottom = np.array([grasp.bottom.x, grasp.bottom.y, grasp.bottom.z])
#         approach = np.array([grasp.approach.x, grasp.approach.y, grasp.approach.z ])
#         binormal = np.array([grasp.binormal.x, grasp.binormal.y, grasp.binormal.z])
#         hand_outer_diameter = 0.12
#         hw = 0.5*hand_outer_diameter
#         finger_width = 0.01
#         left_bottom = bottom - (hw - 0.5 * finger_width) * binormal
#         right_bottom = bottom + (hw - 0.5 * finger_width) * binormal
#         base_center = left_bottom + 0.5 * (right_bottom - left_bottom) - 0.01 * approach
#         approach_center = base_center - 0.06 * approach

#         approach_4 = np.array([grasp.approach.x, grasp.approach.y, grasp.approach.z , approach_center[0]])
#         binormal_4 = np.array([grasp.binormal.x, grasp.binormal.y, grasp.binormal.z, approach_center[1]])
#         axis_4 = np.array([grasp.axis.x, grasp.axis.y, grasp.axis.z, approach_center[2]])

# _robot.enable_interactive()


#     def segment_object(self, object_pos_head_frame):
#         rospy.wait_for_service('/object_segmentation')
        response=None
        try:
            response = self.segment_object_service(
                object_pos_head_frame[0],
                object_pos_head_frame[1],
                object_pos_head_frame[2],
            )
            # segment_object_service = rospy.ServiceProxy('/object_segmentation', SegmentObject)
            # response = segment_object_service(object_pos_head_frame[0],
            #                                   object_pos_head_frame[1],
            #                                   object_pos_head_frame[2])
            
        except rospy.ServiceException as e:
            rospy.logerr("%s: Service call failed: %s" % (self._action_name, e))
            return False

        return response.result

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

    def _execute_cb(self, goal_msg):
        """
        Action server callback for PickUpObjectAction
        """
        _result = PickUpObjectResult()

        goal_tf = goal_msg.goal_tf
        rospy.loginfo("%s: Requested to pick up tf %s" % (self._action_name, goal_tf))

        # Attempt to find transform from hand frame to goal_tf
        (trans, lookup_time) = self.lookup_transform(self.HAND_FRAME, goal_tf)
        
        # For simulation
        num_tf_fails = 0
        while goal_tf is None and num_tf_fails <= NUM_TF_FAILS:
            (trans, lookup_time) = self.lookup_transform(self.HAND_FRAME, goal_tf)                
            num_tf_fails += 1

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
                min=Point(goal_x - 1.0, goal_y - 1.0, goal_z - 1.0),
                max=Point(goal_x + 1.0, goal_y + 1.0, goal_z + 1.0),
            )

            # TODO this is a hard-coded ~10cm box 
            box=0.2
            object_bounding_box = BoundingBox(
                min=Point(goal_x - box, goal_y - box, goal_z - box - 0.1), #0.1 is z offset for simulation only
                max=Point(goal_x + box, goal_y + box, goal_z + box - 0.1),
            )

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

        rospy.loginfo('Is using collision: %s' % (self.use_collision_map))
        rospy.loginfo('Is using synthesis: %s' % (self.use_grasp_synthesis))
        if self.use_collision_map:
            self.whole_body.collision_world = self.collision_world
        else:
            self.whole_body.collision_world = None

        try:
            if self.use_grasp_synthesis:
                print('Starting grasp synthesis')

                # Segment the object point cloud first
                object_position_head_frame = self.get_head_frame_object_pose(goal_tf)

                # object_position_head_frame = self.get_head_frame_object_pose(self.goal_object)
                print('object_position_head_frame:',object_position_head_frame)

                # Call segmentation (lasts 10s)
                seg_response = self.segment_grasp_target_object(object_position_head_frame)
                print('segment response:',seg_response)
                
                self.tts.say('I am trying to calculate the best possible grasp position')
                rospy.sleep(1)                

                pre_grasp_pos = False
                grasp_trial = 0
                while not pre_grasp_pos:
                    try:
                        rospy.loginfo("Grasping trial %s" % grasp_trial)
                        # Get the best grasp - returns the pose-tuple in the head-frame
                        grasp = self.get_grasp(index=grasp_trial)
                        grasp_trial += 1
                        rospy.loginfo("%s: Moving to pre-grasp position." % (self._action_name))
                        self.tts_say("I will now move into the grasp position")
                        rospy.sleep(1)
                        self.whole_body.move_end_effector_pose(
                            grasp, "head_rgbd_sensor_rgb_frame"
                        )
                        pre_grasp_pos = True
                    except:
                        pass

            else:
                # Move to pregrasp
                rospy.loginfo("%s: Calculating grasp pose." % (self._action_name))

                hand_pose = self.get_default_grasp_pose(
                    goal_tf,
                    relative=chosen_pregrasp_pose
                )

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
            self.gripper.apply_force(self.DEFAULT_GRASP_FORCE)
            rospy.loginfo("%s: Object grasped." % self._action_name)

            return True

        except Exception as e:
            rospy.loginfo("%s: Encountered exception %s." % (self._action_name, str(e)))
            self.whole_body.collision_world = None
            self.abandon_action()
            return False

    def get_head_frame_object_pose(self, object_tf):
        found_trans = False
        listen = tf.TransformListener()
        rospy.sleep(3)
        while not found_trans:
            try:
                t = listen.getLatestCommonTime("/head_rgbd_sensor_rgb_frame", object_tf)
                (trans, rot) = listen.lookupTransform('/head_rgbd_sensor_rgb_frame', object_tf, t)
                found_trans = True
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                if self._as.is_preempt_requested():
                    rospy.loginfo('%s: Preempted. Moving to go and exiting.' % self._action_name)
                self.whole_body.move_to_go()
                self._as.set_preempted()
                continue

        return np.array([trans[0], trans[1], trans[2]])

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
        
    def execute_cb_1(self, goal_msg):
        
        self.tts.say("Finding stable view of object")
        rospy.sleep(5)

        _result = PickUpObjectResult()
        # _result.result = False
        is_preempted = False
        # Currently doesn't do anything other than relay to another topic
        rospy.Subscriber("known_object_pre_filter", CollisionObject, self.collision_callback)

        goal_tf_in = goal_msg.goal_tf
        goal_tf = None

        num_tf_fails = 0
        while goal_tf is None and num_tf_fails <= NUM_TF_FAILS:
            goal_tf = self.get_similar_tf(goal_tf_in)
                

            if self._as.is_preempt_requested():
                rospy.loginfo('%s: Preempted. Moving to go and exiting.' % self._action_name)
                self.whole_body.move_to_go()
                self._as.set_preempted()
                is_preempted = True
                return


            if goal_tf is None:
                num_tf_fails += 1
                rospy.loginfo('{0}: Found no similar tf frame. Trying again'.format(self._action_name))

        # Set aborted if we couldn't find a TF frame
        if num_tf_fails > NUM_TF_FAILS:
            rospy.logerr("Couldn't find similar tf frame.")
            self._as.set_aborted()
            return

        if is_preempted:
            return

        # Found the goal tf so proceed to pick up
        rospy.loginfo('{0}: Choosing tf frame "{1}".'.format(self._action_name, str(goal_tf)))
        self.set_goal_object(goal_tf)
        obj_dist = self.get_object_distance(self.goal_object)
        rospy.loginfo('{0}: Distance to object is "{1:.2f}"m.'.format(self._action_name, obj_dist))
        self.tts.say('I can see the object and it is "{:.2f}" metres away.'.format(obj_dist))
        rospy.sleep(1)

        if self.goal_object == 'postcard':
            grasp_type = 'suction'
        else:
            grasp_type = 'grab'
            chosen_pregrasp_pose = geometry.pose(z=-0.08, ek=0)
            chosen_grasp_pose = geometry.pose(z=0.06)

        # ------------------------------------------------------------------------------
        # Check the object is in sight
        found_marker = self.check_for_object(self.goal_object)

        if not found_marker:
            rospy.logerr("Unable to find TF frame...")
            self._as.set_aborted()
            return

        # Look at the object - this is to make sure that we get all of the necessary collision map
        # rospy.loginfo('%s: Moving head to look at the object.' % self._action_name)
        # self.whole_body.gaze_point(ref_frame_id=self.goal_object)

        # Set collision map
        if self.use_collision_map:
            self.tts.say("I am now evaluating my environment so that I don't collide with anything.")
            rospy.sleep(1)
            rospy.loginfo('%s: Getting Collision Map.' % self._action_name)
        
            self.collision_world = self.collision_mapper.get_collision_map()
            rospy.loginfo('%s: Collision Map generated.' % self._action_name)

            rospy.loginfo('%s: Pruning the collision map.' % self._action_name)
            self.collision_mod(self.collision_msg)

    def get_grasp(self,index=0):
        """
        For grasp pose synthesis.
        """

        while not len(self.grasps) > 0:
            if len(self.grasps) > 0:
                rospy.loginfo("Received %d grasps.", len(self.grasps))
                break

        grasp = self.grasps[index]  # grasps are sorted in descending order by score
        rospy.loginfo(
            "%s: Selected grasp with score:: %s" % (self._action_name, str(grasp.score))
        )

        # if grasp_type == 'suction':
        #     self.suck_object()
        # else:        
        #     self.tts.say("I will now pick up the object")
        #     rospy.sleep(1)
        #     grab_success = self.grab_object(chosen_pregrasp_pose, chosen_grasp_pose)

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
    server = PickUpObjectAction("pick_up_object", use_collision_map=True, use_grasp_synthesis=True)
    rospy.spin()
