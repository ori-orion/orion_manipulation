#! /usr/bin/env python3

import rospy
import actionlib
import tf
import math
import hsrb_interface
import hsrb_interface.geometry as geometry

import orion_actions.msg as msg

# Enable robot interface
from hsrb_interface import robot as _robot
_robot.enable_interactive()


class PourIntoAction(object):

    def __init__(self, name):
        self._action_name = 'pour_into'
        self._as = actionlib.SimpleActionServer(self._action_name, msg.PourIntoAction,
                                                execute_cb=self.execute_cb, auto_start=False)
        self._as.start()
        rospy.loginfo('%s: Action name is: %s' % (self._action_name, name))

        # Preparation for using the robot functions
        self.robot = hsrb_interface.Robot()
        self.whole_body = self.robot.try_get('whole_body')
        self.gripper = self.robot.try_get('gripper')
        self.whole_body.end_effector_frame = 'hand_palm_link'
        self.whole_body.looking_hand_constraint = True
        self.tts = self.robot.try_get('default_tts')
        self.tts.language = self.tts.ENGLISH

        self.whole_body.planning_timeout = 20.0  # Increase planning timeout. Default is 10s

        # Constants for pouring operation
        self._POUR_SPEED = 0.2
        self._DEFAULT_POUR_HEIGHT = 0.05
        self.pour_pose = geometry.pose(y=-0.20, z=-0.05, ek=-1.57)

        rospy.loginfo('%s: Initialised. Ready for clients.' % self._action_name)

    def execute_cb(self, goal_msg):
        # Messages for feedback / results
        _result = msg.PourIntoResult()
        _result.result = False

        is_preempted = False

        # Get the tf of the object to be poured into, from the message
        goal_tf_in = goal_msg.goal_tf_frame
        goal_tf = None

        while goal_tf is None:
            goal_tf = self.get_similar_tf(goal_tf_in)

            # Give opportunity to preempt
            if self._as.is_preempt_requested():
                rospy.loginfo('%s: Preempted. Moving to go and exiting.' % self._action_name)
                self.whole_body.move_to_go()
                self._as.set_preempted()
                is_preempted = True
                return

            if goal_tf is None:
                rospy.loginfo('{0}: Found no similar tf frame. Trying again'.format(self._action_name))

        if is_preempted:
            return

        rospy.loginfo('{0}: Choosing tf frame "{1}".'.format(self._action_name, str(goal_tf)))

        try:
            rospy.loginfo('%s: Executing, pouring into object at %s.' % (self._action_name, goal_tf))
            self.tts.say("I will pour what I am holding into the specified object.")
            rospy.sleep(1)

            self.whole_body.move_to_neutral()
            rospy.sleep(1)

            # Move grasper over the object to pour into
            self.whole_body.move_end_effector_pose(self.pour_pose, goal_tf)

            # Attempt to start pour
            self.whole_body.move_end_effector_by_arc(geometry.pose(x=self._DEFAULT_POUR_HEIGHT), math.radians(90.0), ref_frame_id='hand_palm_link')

            # Delay to make sure pouring is complete
            rospy.sleep(4)

            self.tts.say("Poured successfully. I will now returning to position.")
            rospy.sleep(1)

            # Return to "go" pose
            rospy.loginfo('%s: Returning to go pose.' % (self._action_name))
            self.whole_body.move_to_go()

            rospy.loginfo('%s: Succeeded' % self._action_name)
            _result.result = True
            self._as.set_succeeded(_result)

        except Exception as e:
            rospy.loginfo('{0}: Encountered exception {1}.'.format(self._action_name, str(e)))
            self.tts.say("I encountered a problem. Returning to go position and aborting pour.")
            rospy.sleep(2)
            rospy.loginfo('%s: Returning to go pose.' % (self._action_name))
            self.whole_body.move_to_go()
            self._as.set_aborted()

    def get_similar_tf(self, tf_frame):
        listen = tf.TransformListener()
        rospy.sleep(3)
        all_frames = listen.getFrameStrings()
        for object_tf in all_frames:
            rospy.loginfo('%s: Found tf frame: %s' % (self._action_name, object_tf))
            if tf_frame.split('_')[-1] in object_tf.split('-')[0]:
                return object_tf


if __name__ == '__main__':
    rospy.init_node('pour_into_server')
    server = PourIntoAction(rospy.get_name())
    rospy.spin()
