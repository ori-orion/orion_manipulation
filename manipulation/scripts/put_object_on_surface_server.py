#! /usr/bin/env python3
""" Action server for putting objects on a surface in front.
Currently in development
"""

import rospy
import actionlib
import hsrb_interface
import hsrb_interface.geometry as geometry

# Enable robot interface
from hsrb_interface import robot as _robot

_robot.enable_interactive()

from actionlib_msgs.msg import GoalStatus
from manipulation.manipulation_header import ManipulationAction
from orion_actions.msg import *
from point_cloud_filtering.srv import SegmentSurface
from geometry_msgs.msg import Point
import tf
import numpy as np


class PutObjectOnSurfaceAction(ManipulationAction):

    GOAL_OBJECT_TF_TIMEOUT = 20
    PREGRASP_POSE = geometry.pose(z=-0.08, ek=0)  # Relative to gripper
    GRASP_POSE = geometry.pose(z=0.06)  # Relative to gripper

    
    def __init__(self, action_name, action_msg_type=orion_actions.msg.PutObjectOnSurfaceAction, use_collision_map=True):
              
        super(PutObjectOnSurfaceAction, self).__init__(
            action_name, action_msg_type, use_collision_map
        )
        rospy.wait_for_service("/surface_segmentation")
        self.segment_surface_service = rospy.ServiceProxy(
                "/surface_segmentation", SegmentSurface
            )

        self.placeholder = None
        self.placeholder_sub = rospy.Subscriber('/placeholder', Point, self.placeholder_callback)
        
        rospy.loginfo('%s: Initialised. Ready for clients.' % self._action_name)

    def placeholder_callback(self, msg):
        self.placeholder = msg
        

    def segment_surface(self, object_pos_head_frame):
        """
        Request the object_segmentation node to publish a segmented point cloud at a
        specified location in the head RGBD frame, containing the object we wish to
        generate grasp poses for.
        The grasp pose synthesis node will input this point cloud and publish a list of
        candidate poses.
        """
        try:
            response = self.segment_surface_service(
                object_pos_head_frame[0],
                object_pos_head_frame[1],
                object_pos_head_frame[2],
            )
            return True

        except rospy.ServiceException as e:
            rospy.logerr("%s: Service call failed: %s" % (self._action_name, e))
            return False


    ## THE SAME IN PICK UP SERVER
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
        # odom_to_hand = geometry.multiply_tuples(odom_to_ref, relative)
        return odom_to_ref

    def _execute_cb(self, goal_msg):
        _result = PutObjectOnSurfaceResult()
        _result.result = False

        goal_tf = goal_msg.goal_tf
        rospy.loginfo("%s: Requested to put object on surface %s" % (self._action_name, goal_tf))

        # Give opportunity to preempt
        if self._as.is_preempt_requested():
            rospy.loginfo('%s: Preempted' % self._action_name)
            self._as.set_preempted()
            return

        try:
            self.whole_body.end_effector_frame = self.HAND_FRAME
            # rospy.loginfo('%s: Placing gripper close to surface in front' % (self._action_name))
            # self.tts.say("I will place the object on the surface in front of me.")
            # rospy.sleep(2)
            self.whole_body.move_to_neutral()
            rospy.sleep(1)

            # Attempt to find transform from hand frame to goal_tf
            (trans, lookup_time) = self.lookup_transform(self.HAND_FRAME, goal_tf)
            if trans is None:
                rospy.logerr("Unable to find TF frame")
                self.tts_say("I don't know the surface frame you want to put object down.")
                self.abandon_action()
                return
            
            # Look at the object - make sure that we get all of the necessary collision map
            rospy.loginfo("%s: Moving head to look at the location." % self._action_name)
            self.look_at_object(goal_tf)
            if (rospy.Time.now() - lookup_time).to_sec() > self.GOAL_OBJECT_TF_TIMEOUT:
                rospy.logerr("Most recent published goal TF frame is too old ({} seconds old)".format((rospy.Time.now() - lookup_time).to_sec()))
                self.tts_say("I can't see the object you want picked up.")
                self.abandon_action()
                return


            surface_position = self.get_head_frame_object_pose(goal_tf)
            rospy.loginfo("Segmenting surface..")
            # Segment surface
            self.segment_surface(surface_position)
            rospy.loginfo("Finished segmentation")
            # rospy.sleep(1)
            if self.placeholder is not None:
                rospy.loginfo(self.placeholder)
                # self.whole_body.move_end_effector_pose(geometry.pose(x=self.placeholder.x, y=self.placeholder.y, z=self.placeholder.z), self.RGBD_CAMERA_FRAME)
                (trans, lookup_time) = self.lookup_transform(self.HAND_FRAME, self.RGBD_CAMERA_FRAME)
                self.whole_body.move_end_effector_pose(geometry.pose(x=self.placeholder.x+trans.translation.x, y=self.placeholder.y+trans.translation.y, z=self.placeholder.z+trans.translation.z), self.HAND_FRAME)
            else:
                rospy.loginfo("Failed to find generated placeholder position from surface segmentation. Moving to the input frame instead..")
                hand_pose = self.get_default_grasp_pose(
                        goal_tf,
                        relative=self.PREGRASP_POSE
                    )

            # Place object of surface
                # self.whole_body.move_end_effector_pose(geometry.pose(x=0, y=0, z=0.2), 'hand_palm_link')
                self.whole_body.move_end_effector_pose(geometry.pose(x=trans.translation.x, y=trans.translation.y, z=trans.translation.z), self.HAND_FRAME)
                # self.whole_body.move_end_effector_pose(hand_pose, self.ODOM_FRAME)
                # self.whole_body.move_end_effector_pose(self.GRASP_POSE, self.whole_body.end_effector_frame)

            # Let go of the object
            rospy.sleep(1)
            rospy.loginfo('%s: Opening gripper.' % (self._action_name))
            self.gripper.command(1.2)
            self.tts.say("Object placed successfully. Returning to go position.")
            rospy.sleep(2)

            # # Move the gripper back a bit then return to go
            self.whole_body.linear_weight = 100
            self.whole_body.move_end_effector_pose(geometry.pose(z=-0.2), 'hand_palm_link')
            rospy.loginfo('%s: Returning to go pose.' % (self._action_name))
            self.whole_body.move_to_go()

            rospy.loginfo('%s: Succeeded' % self._action_name)
            _result.result = True
            self._as.set_succeeded(_result)

        except Exception as e:
            rospy.loginfo('{0}: Encountered exception {1}.'.format(self._action_name, str(e)))
            self.tts.say("I encountered a problem. Returning to go position and aborting placement.")
            rospy.sleep(2)
            rospy.loginfo('%s: Returning to go pose.' % (self._action_name))
            self.whole_body.move_to_go()
            self._as.set_aborted()


if __name__ == '__main__':
    rospy.init_node('put_object_on_surface_server')
    server = PutObjectOnSurfaceAction('put_object_on_surface', use_collision_map=False)
    rospy.spin()
