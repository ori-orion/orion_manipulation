#! /usr/bin/env python3
""" Test plane detection.
Requires surface_segmentation_node to be running to provide /detect_surface.
"""

import sys
import rospy

import hsrb_interface.geometry as geometry
from geometry_msgs.msg import Transform, Point
from manipulation.manipulation_header import ManipulationAction
from manipulation.srv import FindPlacementOnEmptySurface

# Enable robot interface
from hsrb_interface import robot as _robot
_robot.enable_interactive()


class FindPlacementOnEmptySurfaceTester(ManipulationAction):

    def __init__(self):

        super(FindPlacementOnEmptySurfaceTester, self).__init__(
            action_name="test_empty_placement_finder",
            action_msg_type=None,
            use_collision_map=False,
            tts_narrate=False,
            prevent_motion=False,
        )

        rospy.loginfo("%s: Waiting for /FindPlacementOnEmptySurface" % (self._action_name))
        rospy.wait_for_service("/FindPlacementOnEmptySurface")
        self.find_placement_service = rospy.ServiceProxy(
            "/FindPlacementOnEmptySurface", FindPlacementOnEmptySurface
        )
        rospy.loginfo("%s: Subscribed to /FindPlacementOnEmptySurface" % (self._action_name))

    def execute(self, goal_tf, iterative=False):
        rospy.loginfo("%s: Moving head to look at the location." % self._action_name)
        self.look_at_object(goal_tf)

        (rgbd_goal_transform, _) = self.lookup_transform(self.RGBD_CAMERA_FRAME, goal_tf, rospy.Duration(5))

        if rgbd_goal_transform is None:
            rospy.logerr("%s: Unable to find TF frame." % self._action_name)
            return

        rospy.loginfo("The hunt begins...")
        position = self.find_placement(rgbd_goal_transform)

        if position is None:
            rospy.logerr("Unable to find a placement")
            return

        rospy.loginfo("%s: Received a placement option" % (self._action_name))

        pose = geometry.Pose(
            geometry.Vector3(
                position[0], position[1], position[2]
            ),
            geometry.Quaternion(0, 0, 0, 1), # TODO: Use a better Quaternion
        )

        self.publish_goal_pose_tf(pose, self.MAP_FRAME, "placement")
        return

    def find_placement(self, plane_search_transform_in_head_frame):
        try:
            res = self.find_placement_service(
                Point(0.1, 0.1, 0.05), # dimension of object
                0.3, # max height from surface

                plane_search_transform_in_head_frame,
                10.0,  # EPS plane search angle tolerance in degrees
                0.5,  # Box crop size to search for plane in. Axis aligned w/ head frame.
                0
            )
            if res.success:
                return res.position
            else:
                return None

        except rospy.ServiceException as e:
            rospy.logerr("%s: Service call failed: %s" % (self._action_name, e))
            return None


if __name__ == "__main__":
    rospy.init_node("test_surface_detection")
    tester = FindPlacementOnEmptySurfaceTester()

    if len(sys.argv) == 2:
        goal_tf = sys.argv[1]
        tester.execute(goal_tf)

    if len(sys.argv) == 3:
        goal_tf = sys.argv[1]
        iterative = int(sys.argv[2])
        iterative = (iterative > 0)
        tester.execute(goal_tf, iterative)

    else:
        print("TFs available to test in manipulation_test_sim world:")
        print("  - surface_placement_l")
        print("  - surface_high_1")
        print("  - surface_low_1")
        print("  - surface_floor_1")
        print("  - surface_floor_2")
        while not rospy.is_shutdown():
            goal_tf = input('Enter a tf name to detect surfaces at: ')
            tester.execute(goal_tf)
            print()
