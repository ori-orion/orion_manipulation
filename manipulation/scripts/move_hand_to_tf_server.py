#! /usr/bin/env python
""" Development action server for moving hand to a given tf frame.
Not used now.
"""
__author__ = "Mark Finean"
__email__ = "mfinean@robots.ox.ac.uk"

# import roslib; roslib.load_manifest('manipulation')
import hsrb_interface
import hsrb_interface.geometry as geometry
import numpy as np
import rospy
import actionlib

from manipulation.manipulation_header import *
from tmc_manipulation_msgs.msg import CollisionObject
from manipulation.msg import *

class MoveHandToTfAction(object):
    # create messages that are used to publish feedback/result
    _feedback = MoveHandToTfActionFeedback()
    _result = MoveHandToTfActionResult()

    def __init__(self, name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, manipulation.msg.MoveHandToTfAction,
                                                execute_cb=self.execute_cb, auto_start=False)
        self._as.start()

        # Put bounds about the tf frrmame of object to remove from map
        self.exclusion_bounds = np.array([0.07, 0.07, 0.07])

        # Preparation for using the robot functions
        self.robot = hsrb_interface.Robot()
        self.whole_body = self.robot.try_get('whole_body')
        self.omni_base = self.robot.try_get('omni_base')
        self.collision_world = self.robot.try_get('global_collision_world')
        self.gripper = self.robot.try_get('gripper')

        self._HAND_TF = 'hand_palm_link'
        self._GRASP_FORCE = 0.2

        # Set the grasp poses
        self.pregrasp_pose = geometry.pose(z=-0.05, ek=-1.57)
        self.grasp_pose = geometry.pose(z=0.02)


    def callback(msg):
        # Get the message
        message = msg

        # Find which boxes to removes
        inds_to_remove = []
        for i in range(len(message.poses)):
            pose = message.poses[i]
            if (pose.position.x <= upper_bounds[0] and pose.position.x >= lower_bounds[0] and
                    pose.position.y <= upper_bounds[1] and pose.position.y >= lower_bounds[1] and
                    pose.position.z <= upper_bounds[2] and pose.position.z >= lower_bounds[2]):
                inds_to_remove.append(i)

        # Remove the boxes
        for index in sorted(inds_to_remove, reverse=True):
            del message.poses[index], message.shapes[index]

        # Publish the filtered message
        pub.publish(message)
        print("Message published")


    def execute_cb(self, goal_msg):
        success = True
        goal_tf = goal_msg.goal_tf

        # publish info to the console for the user
        rospy.loginfo('%s: Executing, moving hand to %s.' % ( self._action_name, goal_tf))

        global pub, lower_bounds, upper_bounds

        rospy.loginfo('%s: Moving into position.' % (self._action_name))
        self.omni_base.go_abs(1.3370214380590735, 0.40718077536695114, 1.57)

        # ---------------------------------- Now begin the actual actions --------------------
        self.whole_body.move_to_go()

        # Open gripper
        rospy.loginfo('%s: Opening gripper.' % (self._action_name))
        self.gripper.command(1.2)

        # Follow the gripper
        self.whole_body.looking_hand_constraint = True

        # Set collision map
        rospy.loginfo('%s: Getting Collision Map.' % (self._action_name))
        get_collision_map(self.robot)
        rospy.loginfo('%s: Collision Map generated.' % (self._action_name))

        # Get the object pose to subtract from collision map
        rospy.loginfo('%s: Getting object pose.' % (self._action_name))
        goal_object_pose = get_object_pose(goal_tf)
        upper_bounds = goal_object_pose + self.exclusion_bounds
        lower_bounds = goal_object_pose - self.exclusion_bounds

        rospy.loginfo('%s: Bounds have been set' % self._action_name)
        print("Bounds have been set")

        # Remove object from map and publish to correct topic
        rospy.loginfo('%s: Modifying collision map.' % (self._action_name))
        pub = rospy.Publisher('known_object', CollisionObject, queue_size=1)
        rospy.Subscriber("known_object_pre_filter", CollisionObject, self.callback)
        rospy.sleep(2)

        rospy.loginfo('%s: Finding object.' % (self._action_name))
        # Set up listener to find the bottle
        check_for_object(goal_tf)

        rospy.loginfo('%s: Executing grasp procedure.' % (self._action_name))
        # Turn on collision checking
        self.whole_body.collision_world = self.collision_world

        print("Moving to object...")
        self.whole_body.move_end_effector_pose(self.pregrasp_pose, goal_tf)

        # Turn off collision checking to get close and grasp
        self.whole_body.collision_world = None
        self.whole_body.move_end_effector_pose(self.grasp_pose, 'hand_palm_link')

        # Specify the force to grasp
        self.gripper.apply_force(self._GRASP_FORCE)
        rospy.sleep(2.0)
        rospy.loginfo('%s: Object grasped.' % self._action_name)
        print("Object grasped.")

        # Turn collision checking back on
        self.whole_body.collision_world = self.collision_world
        rospy.sleep(0.5)

        # Now return to moving position
        rospy.loginfo('%s: Try to move to go.' % self._action_name)
        print("Try to move to go.")
        self.whole_body.move_to_go()

        print("Finished")

        if success:
            # self._result.sequence = self._feedback.sequence
            rospy.loginfo('%s: Succeeded' % self._action_name)
            # self._as.set_succeeded(self._result)
            self._as.set_succeeded(1)


if __name__ == '__main__':
    rospy.init_node('move_hand_to_tf_server_node')
    server = MoveHandToTfAction(rospy.get_name())
    rospy.spin()
