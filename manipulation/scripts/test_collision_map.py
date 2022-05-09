#! /usr/bin/env python3

import hsrb_interface
import rospy

from manipulation.manipulation_header import CollisionMapper

from hsrb_interface import robot as _robot
_robot.enable_interactive()


if __name__ == "__main__":

    rospy.init_node("test_node")

    robot = hsrb_interface.Robot()

    collision_mapper = CollisionMapper(robot)
    collision_mapper.build_collision_world()

    print("Done")

    while not rospy.is_shutdown():
        rospy.spin()
