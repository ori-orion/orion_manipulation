#! /usr/bin/env python3
""" Action server for opening the bin lid.
"""
__author__ = "Mark Finean"
__email__ = "mfinean@robots.ox.ac.uk"

import hsrb_interface
import rospy
import actionlib
import numpy as np
import tf
import hsrb_interface.geometry as geometry
import tf.transformations as T
import math

# from manipulation.manipulation_header import CollisionMapper
from actionlib_msgs.msg import GoalStatus
from orion_actions.msg import *


from hsrb_interface import robot as _robot
_robot.enable_interactive()

# For grasp synthesis
from point_cloud_filtering.srv import DetectBinHandle
from gpd.msg import GraspConfigList

class PickUpObjectAction(object):

    def __init__(self, name):
        self._action_name = 'open_bin_lid'
        self._as = actionlib.SimpleActionServer(self._action_name, 	orion_actions.msg.OpenBinLidAction,
                                                execute_cb=self.execute_cb, auto_start=False)
        self._as.start()
        rospy.loginfo('%s: Action name is: %s' % (self._action_name, name))

        # Preparation for using the robot functions
        self.robot = hsrb_interface.Robot()
        self.whole_body = self.robot.try_get('whole_body')
        self.omni_base = self.robot.try_get('omni_base')
        self.collision_world = self.robot.try_get('global_collision_world')
        self.gripper = self.robot.try_get('gripper')
        self.whole_body.end_effector_frame = 'hand_palm_link'
        self.whole_body.looking_hand_constraint = True
        self.tts = self.robot.try_get('default_tts')
        self.tts.language = self.tts.ENGLISH

        self.collision_mapper = CollisionMapper(self.robot)

        self._CONNECTION_TIMEOUT = 15.0 # Define the vacuum timeouts
        self._HAND_TF = 'hand_palm_link'
        self._GRASP_FORCE = 2.0

        self.whole_body.planning_timeout = 20.0 # Increase planning timeout. Default is 10s
        self.whole_body.tf_timeout = 10.0 # Increase tf timeout. Default is 5s

        # Set up publisher for the collision map
        # self.pub = rospy.Publisher('known_object', CollisionObject, queue_size=1)
        self.goal_pose_br = tf.TransformBroadcaster()

        self.grasps = None
        self.use_grasp_synthesis = False

        self.goal_object = None

        rospy.loginfo('%s: Initialised. Ready for clients.' % self._action_name)

    def grasp_callback(self, msg):
        self.grasps = msg.grasps

    def get_grasp(self):

        while not len(self.grasps) > 0:
            if len(self.grasps) > 0:
                rospy.loginfo('Received %d grasps.', len(self.grasps))
                break

        grasp = self.grasps[0] # grasps are sorted in descending order by score
        rospy.loginfo('%s: Selected grasp with score:: %s' % (self._action_name, str(grasp.score)))

        # This gives the approach point correctly
        bottom = np.array([grasp.bottom.x, grasp.bottom.y, grasp.bottom.z])
        approach = np.array([grasp.approach.x, grasp.approach.y, grasp.approach.z ])
        binormal = np.array([grasp.binormal.x, grasp.binormal.y, grasp.binormal.z])
        hand_outer_diameter = 0.12
        hw = 0.5*hand_outer_diameter
        finger_width = 0.01
        left_bottom = bottom - (hw - 0.5 * finger_width) * binormal
        right_bottom = bottom + (hw - 0.5 * finger_width) * binormal
        base_center = left_bottom + 0.5 * (right_bottom - left_bottom) - 0.01 * approach
        approach_center = base_center - 0.06 * approach

        approach_4 = np.array([grasp.approach.x, grasp.approach.y, grasp.approach.z , approach_center[0]])
        binormal_4 = np.array([grasp.binormal.x, grasp.binormal.y, grasp.binormal.z, approach_center[1]])
        axis_4 = np.array([grasp.axis.x, grasp.axis.y, grasp.axis.z, approach_center[2]])

        R = np.array([axis_4, -binormal_4, approach_4, [0, 0, 0, 1]])
        q = T.quaternion_conjugate(T.quaternion_from_matrix(R))

        return geometry.Pose(geometry.Vector3(approach_center[0], approach_center[1], approach_center[2]), geometry.Quaternion(q[0], q[1], q[2], q[3]))


    def get_bin_handle_poses(self):
        rospy.wait_for_service('/bin_handle_detection')
        try:
            detect_handle_service = rospy.ServiceProxy('/bin_handle_detection', DetectDrawerHandles)
            response = detect_handle_service(True)
            # response should contain and array of x,y,z coords
        except rospy.ServiceException as e:
            print ("Service call failed: %s" % e)

        return response

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

    def get_goal_pose(self, relative=geometry.pose()):
        rospy.loginfo('%s: Trying to lookup goal pose...' % self._action_name)
        foundTrans = False
        while not foundTrans:
            try:
                odom_to_ref_pose = self.whole_body._lookup_odom_to_ref(self.goal_object)
                foundTrans = True
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                if self._as.is_preempt_requested():
                    rospy.loginfo('%s: Preempted. Moving to go and exiting.' % self._action_name)
                    self.whole_body.move_to_go()
                    self._as.set_preempted()
                    # TO DO - introudce a self.to_preempt = True
                    return

        odom_to_ref = geometry.pose_to_tuples(odom_to_ref_pose)
        odom_to_hand = geometry.multiply_tuples(odom_to_ref, relative)

        rospy.loginfo('%s: Calculated the goal pose.' % self._action_name)

        pose_msg = geometry.tuples_to_pose(odom_to_hand)

        eulers = T.euler_from_quaternion([pose_msg.orientation.x, pose_msg.orientation.y, pose_msg.orientation.z, pose_msg.orientation.w])

        return geometry.pose(x=pose_msg.position.x,
                             y=pose_msg.position.y,
                             z=pose_msg.position.z,
                             ei=eulers[0],
                             ej=eulers[1],
                             ek=eulers[2])

    def get_object_pose(self, object_tf):
        found_trans = False
        listen = tf.TransformListener()
        rospy.sleep(3)
        while not found_trans:
            try:
                t = listen.getLatestCommonTime("/map", object_tf)
                (trans, rot) = listen.lookupTransform('/map', object_tf, t)
                found_trans = True
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                if self._as.is_preempt_requested():
                    rospy.loginfo('%s: Preempted. Moving to go and exiting.' % self._action_name)
                    self.whole_body.move_to_go()
                    self._as.set_preempted()
                    return

        return np.array([trans[0], trans[1], trans[2]])

    def set_goal_object(self, obj):
        self.goal_object = obj

    def collision_mod(self, msg):
        if self.goal_object:
            object_pose = self.get_object_pose(self.goal_object)
            upper_bound = object_pose + np.array([0.7, 0.7, 1])
            lower_bound = object_pose - np.array([0.7, 0.7, 1])

            object_upper_bound = object_pose + np.array([0.05, 0.05, 0.03])
            object_lower_bound = object_pose - np.array([0.05, 0.05, 0.03])

            # Get the message
            message = msg
            rospy.loginfo('%s: Removing excess collision space.' % self._action_name)

            # Find which boxes to removes
            inds_to_remove = []
            for i in range(len(message.poses)):
                pose = message.poses[i]
                pose_arr = np.array([pose.position.x,pose.position.y,pose.position.z])

                # remove excess environment
                if not(np.all(pose_arr <= upper_bound) and np.all(pose_arr >= lower_bound)):
                    inds_to_remove.append(i)

                # approx remove the object
                if np.all(pose_arr <= object_upper_bound) and np.all(object_lower_bound >= lower_bound):
                    inds_to_remove.append(i)

            # Remove the boxes
            for index in sorted(inds_to_remove, reverse=True):
                del message.poses[index], message.shapes[index]

            # Publish the filtered message
            self.pub.publish(message)
        else:
            self.pub.publish(msg)

    def collision_callback(self, msg):
        self.collision_msg = msg


    def grab_handle(self, chosen_pregrasp_pose, chosen_grasp_pose):
        self.whole_body.end_effector_frame = 'hand_palm_link'

        rospy.loginfo('%s: Opening gripper.' % (self._action_name))
        self.gripper.command(1.2)
        self.whole_body.collision_world = self.collision_world

        # Segment the object point cloud first
        object_position_head_frame = self.get_head_frame_object_pose(self.goal_object)

        # Call segmentation (lasts 10s)
        seg_response = self.get_bin_handle_poses()

        self.tts.say('I am trying to calculate the best possible grasp position')
        rospy.sleep(1)
        # Get the best grasp - returns the pose-tuple in the head-frame
        grasp = self.get_grasp()

        rospy.loginfo('%s: Moving to pre-grasp position.' % (self._action_name))
        self.tts.say('I will now move into the grasp position')
        rospy.sleep(1)
        self.whole_body.move_end_effector_pose(grasp, 'head_rgbd_sensor_rgb_frame')

        # Turn off collision checking to get close and grasp
        self.tts.say('Turning off collision checking to get closer.')
        rospy.sleep(1)
        rospy.loginfo('%s: Turning off collision checking to get closer.' % (self._action_name))
        self.whole_body.collision_world = None
        rospy.sleep(1)

        # Move to grasp pose
        rospy.loginfo('%s: Moving to grasp position.' % (self._action_name))
        self.tts.say('Moving to grasp position.')
        rospy.sleep(1)
        self.whole_body.move_end_effector_pose(chosen_grasp_pose, self.whole_body.end_effector_frame)

        # Specify the force to grasp
        self.tts.say('Grasping handle.')
        rospy.sleep(1)
        self.gripper.apply_force(self._GRASP_FORCE)
        rospy.loginfo('%s: Handle grasped.' % self._action_name)

        return True


    def finish_position(self):
        self.whole_body.collision_world = self.collision_world
        self.whole_body.move_to_neutral()

        return True

    def execute_cb(self, goal_msg):
        _result = OpenBinLidResult()

        # Currently doesn't do anything other than relay to another topic
        rospy.Subscriber("known_object_pre_filter", CollisionObject, self.collision_callback)

        goal_tf = 'bin_handle'

        # Found the goal tf so proceed to pick up
        self.set_goal_object(goal_tf)

        chosen_pregrasp_pose = geometry.pose(z=-0.08, ek=-math.pi/2)
        chosen_grasp_pose = geometry.pose(z=0.08)

        # Set collision map
        self.tts.say("I am now evaluating my environment so that I don't collide with anything.")
        rospy.sleep(1)
        rospy.loginfo('%s: Getting Collision Map.' % self._action_name)
        self.collision_mapper.get_collision_map()
        rospy.loginfo('%s: Collision Map generated.' % self._action_name)

        rospy.loginfo('%s: Pruning the collision map.' % self._action_name)
        self.collision_mod(self.collision_msg)


        self.tts.say("I will now pick up the object")
        rospy.sleep(1)
        grab_success = self.grab_handle(chosen_pregrasp_pose, chosen_grasp_pose)

        if grab_success == False:
            self.whole_body.move_to_go()
            return

        self.tts.say("Object grasped successfully. Now returning to normal position.")
        rospy.sleep(1)

        # Now return to moving position
        success = self.finish_position()

        if success:
            rospy.loginfo('%s: Succeeded' % self._action_name)
            _result.result = True
            self._as.set_succeeded(_result)
        else:
            rospy.loginfo('%s: Failed' % self._action_name)
            _result.result = False
            self._as.set_aborted()


if __name__ == '__main__':
    rospy.init_node('open_bin_lid_server_node')
    server = OpenBinLidAction(rospy.get_name())
    rospy.spin()
