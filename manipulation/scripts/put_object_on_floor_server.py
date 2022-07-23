#! /usr/bin/env python3
""" Action server for putting objects on the floor.
Hard coded motion to maintain horizontal grasp and place hand close to the floor and open the gripper.
"""

import rospy
import actionlib
import math
import hsrb_interface
import hsrb_interface.geometry as geometry

import orion_actions.msg as msg
from geometry_msgs.msg import WrenchStamped

# Enable robot interface
from hsrb_interface import robot as _robot
_robot.enable_interactive()


def compute_difference(pre_data_list, post_data_list):
    if len(pre_data_list) != len(post_data_list):
        raise ValueError('Argument lists differ in length')

    # Calculate square sum of difference
    square_sums = sum([math.pow(b - a, 2)
                       for (a, b) in zip(pre_data_list, post_data_list)])

    return math.sqrt(square_sums)


class ForceSensorCapture(object):
    """Subscribe and hold force sensor data - Copyright (C) 2016 Toyota Motor Corporation"""

    def __init__(self):
        self._force_data_x = 0.0
        self._force_data_y = 0.0
        self._force_data_z = 0.0

        # Subscribe force torque sensor data
        ft_sensor_topic = '/hsrb/wrist_wrench/raw'
        self._wrist_wrench_sub = rospy.Subscriber(
            ft_sensor_topic, WrenchStamped, self.__hand_force_sensor_cb)

        # Wait for connection
        try:
            rospy.wait_for_message(ft_sensor_topic, WrenchStamped,
                                   timeout=10)
        except Exception as e:
            rospy.logerr(e)

    def get_current_force(self):
        return [self._force_data_x, self._force_data_y, self._force_data_z]

    def __hand_force_sensor_cb(self, data):
        self._force_data_x = data.wrench.force.x
        self._force_data_y = data.wrench.force.y
        self._force_data_z = data.wrench.force.z


class PutObjectOnFloorAction(object):

    def __init__(self, name):
        self._action_name = 'put_object_on_floor'
        self._as = actionlib.SimpleActionServer(self._action_name, msg.PutObjectOnFloorAction,
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

        rospy.loginfo('%s: Initialised. Ready for clients.' % self._action_name)

    def execute_cb(self, goal_msg):
        _result = msg.PutObjectOnFloorResult()
        _result.result = False

        # Give opportunity to preempt
        if self._as.is_preempt_requested():
            rospy.loginfo('%s: Preempted' % self._action_name)
            self._as.set_preempted()
            return

        try:
            rospy.loginfo('%s: Placing gripper close to floor in front' % (self._action_name))
            self.tts.say("I will place the object on the floor in front of me.")
            rospy.sleep(2)
            self.whole_body.linear_weight = 100
            self.whole_body.move_to_neutral()
            rospy.sleep(1)
            self.whole_body.move_end_effector_pose(geometry.pose(x=-0.2, y=0, z=0.3), 'hand_palm_link')
            # self.whole_body.move_end_effector_pose(geometry.pose(x=-0.6, y=0, z=0.2), 'hand_palm_link')

            force_sensor_capture = ForceSensorCapture()

            # Get initial data of force sensor
            pre_grasp_force_list = force_sensor_capture.get_current_force()

            rospy.sleep(1)
            rospy.loginfo('%s: Opening gripper.' % (self._action_name))
            self.gripper.command(1.2)

            post_grasp__force_list = force_sensor_capture.get_current_force()

            # Get the weight of the object and convert newtons to grams
            force_difference = compute_difference(pre_grasp_force_list, post_grasp__force_list)
            weight = math.floor(force_difference / 9.81 * 1000)

            rospy.loginfo('{0}: The weight change by grams is {1}.'.format(self._action_name, str(weight)))
            self.tts.say('{0}: The weight change by grams is {1}.'.format(self._action_name, str(weight)))
            rospy.sleep(3)

            if weight > 100:
                self.tts.say('{0}: I can still feel the object.'.format(self._action_name))
                rospy.sleep(3)
                rospy.loginfo('%s: Now shaking the object off.' % (self._action_name))
                self.tts.say("Now shaking the object off.")
                rospy.sleep(2)
                self.whole_body.move_to_joint_positions({'wrist_roll_joint': -1.8})
                self.whole_body.move_to_joint_positions({'arm_lift_joint': 0.3})
                self.whole_body.move_to_joint_positions({'wrist_roll_joint': 1.8})
                self.whole_body.move_to_joint_positions({'arm_lift_joint': 0.4})
                self.whole_body.move_to_joint_positions({'wrist_roll_joint': 0})
                self.whole_body.move_to_joint_positions({'arm_lift_joint': 0.2})

            self.tts.say("Object placed successfully. Returning to go position.")
            rospy.sleep(2)

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
    rospy.init_node('put_object_on_floor_server')
    server = PutObjectOnFloorAction(rospy.get_name())
    rospy.spin()
