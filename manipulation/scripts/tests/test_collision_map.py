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

    external_bounding_box = BoundingBox(
        min=Point(-100, -100, -100), max=Point(100, 100, 100)
    )

    crop_point = [7.10, 2.05, 0.82]
    crop_bounding_box = BoundingBox(
        min=Point(crop_point[0] - 0.1, crop_point[1] - 0.1, crop_point[2] - 0.1),
        max=Point(crop_point[0] + 0.1, crop_point[1] + 0.1, crop_point[2] + 0.1),
    )

    collision_mapper = CollisionMapper(robot)
    collision_mapper.build_collision_world(external_bounding_box, [crop_bounding_box])

    print("Done")
    rospy.signal_shutdown("Done")
    exit()
