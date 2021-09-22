#!/usr/bin/env python3
"""Get the weight of a grasped object
To integrate with all grasping servers. Adapted code (ForceSensorCapture) from Toyota Motor Corporation
"""
__author__ = "Mark Finean"
__email__ = "mfinean@robots.ox.ac.uk"

import math
import os
import sys

import actionlib
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import WrenchStamped
import rospy

_CONNECTION_TIMEOUT = 10.0

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
                                   timeout=_CONNECTION_TIMEOUT)
        except Exception as e:
            rospy.logerr(e)
            sys.exit(1)

    def get_current_force(self):
        return [self._force_data_x, self._force_data_y, self._force_data_z]

    def __hand_force_sensor_cb(self, data):
        self._force_data_x = data.wrench.force.x
        self._force_data_y = data.wrench.force.y
        self._force_data_z = data.wrench.force.z

def main():
    # Start the force sensor capture
    force_sensor_capture = ForceSensorCapture()

    # Get initial data of force sensor
    pre_grasp_force_list = force_sensor_capture.get_current_force()

    # TO DO - here you would insert the grasping procedure
    rospy.sleep(1.0) # Wait until force sensor data become stable

    post_grasp__force_list = force_sensor_capture.get_current_force()

    # Get the weight of the object and convert newtons to grams
    force_difference = compute_difference(pre_grasp_force_list, post_grasp__force_list)
    weight = math.round(force_difference / 9.81 * 1000, 1)
    print ("The weight is " + str(weight) + 'grams.')
    # Speak object weight in first decimal place

if __name__ == '__main__':
    rospy.init_node('get_object_weight')
    main()