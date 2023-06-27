#! /usr/bin/env python3
""" Test plane detection.
Requires surface_segmentation_node to be running to provide /detect_surface.
"""

import sys
import rospy

from manipulation.manipulation_header import ManipulationAction
from point_cloud_filtering.srv import DetectSurface, DetectSurfaceIterative

# Enable robot interface
from hsrb_interface import robot as _robot
_robot.enable_interactive()


class SurfaceDetectionTester(ManipulationAction):

    def __init__(self):

        super(SurfaceDetectionTester, self).__init__(
            action_name="test_surface_detection",
            action_msg_type=None,
            use_collision_map=False,
            tts_narrate=False,
            prevent_motion=False,
        )

        rospy.loginfo("%s: Waiting for /detect_surface" % (self._action_name))
        rospy.wait_for_service("/detect_surface")
        self.detect_surface_service = rospy.ServiceProxy(
            "/detect_surface", DetectSurface
        )
        rospy.wait_for_service("/detect_surface_iterative")
        self.detect_surface_iterative_service = rospy.ServiceProxy(
            "/detect_surface_iterative", DetectSurfaceIterative
        )
        rospy.loginfo("%s: Subscribed to /detect_surface" % (self._action_name))

    def execute(self, goal_tf, iterative=False):

        rospy.loginfo("%s: Moving head to look at the location." % self._action_name)
        self.look_at_object(goal_tf)

        (rgbd_goal_transform, _) = self.lookup_transform(self.RGBD_CAMERA_FRAME, goal_tf, rospy.Duration(0.3))

        if rgbd_goal_transform is None:
            rospy.logerr("%s: Unable to find TF frame." % self._action_name)
            return

        rospy.loginfo("Segmenting surface...")
        # Segment surface
        if not iterative:
            plane_transform = self.detect_plane_surface(rgbd_goal_transform)

            if plane_transform is None:
                rospy.logerr("Unable to detect a surface")
                return

            rospy.loginfo("%s: Received a plane transform: %s" % (self._action_name, str(plane_transform)))
            self.publish_tf(plane_transform, self.RGBD_CAMERA_FRAME, "placement_plane")
        else:
            rospy.loginfo("Segmenting surface Iteratively...")
            try:
                res = self.detect_surface_iterative_service(
                    rgbd_goal_transform,
                    10.0,  # EPS plane search angle tolerance in degrees
                    0.5,  # Box crop size to search for plane in. Axis aligned w/ head frame.
                )

            except rospy.ServiceException as e:
                rospy.logerr("%s: Service call failed: %s" % (self._action_name, e))
        return

    def detect_plane_surface(self, plane_search_transform_in_head_frame):
        """
        Request the surface_detection node to detect the surface location.
        Z component is used to find the plane axis: Z vector of transform should align
        with where the plane axis is expected to be.
        """
        try:
            res = self.detect_surface_service(
                plane_search_transform_in_head_frame,
                10.0,  # EPS plane search angle tolerance in degrees
                0.5,  # Box crop size to search for plane in. Axis aligned w/ head frame.
            )
            if res.success:
                return res.plane_axis
            else:
                return None

        except rospy.ServiceException as e:
            rospy.logerr("%s: Service call failed: %s" % (self._action_name, e))
            return None


if __name__ == "__main__":
    rospy.init_node("test_surface_detection")
    tester = SurfaceDetectionTester()

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
