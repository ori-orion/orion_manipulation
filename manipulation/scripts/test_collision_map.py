#! /usr/bin/env python3

import hsrb_interface
import rospy

from manipulation.manipulation_header import CollisionMapper
from manipulation.msg import BoundingBox
from geometry_msgs.msg import Point

from hsrb_interface import robot as _robot
_robot.enable_interactive()


if __name__ == "__main__":

    rospy.init_node("test_node")

    robot = hsrb_interface.Robot()

    external_bounding_box = BoundingBox(min=Point(-1000, -1000, -1000), max=Point(1000, 1000, 1000))

    crop_bounding_box = BoundingBox(min=Point(-1000, -1000, -1000), max=Point(1000, 1000, 1000))

    collision_mapper = CollisionMapper(robot)
    collision_mapper.build_collision_world(external_bounding_box, [])

    print("Done")

    while not rospy.is_shutdown():
        rospy.spin()
