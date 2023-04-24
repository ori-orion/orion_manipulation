#! /usr/bin/env python3

import math

import hsrb_interface.geometry as geometry

import rospy
from manipulation.msg import BoundingBox
from geometry_msgs.msg import Transform, TransformStamped, Point
from manipulation.srv import CheckPlacement, FindPlacement, FindPlacementResponse
from manipulation.manipulation_header import ManipulationAction

class PlacementFinder(ManipulationAction):
    def __init__(self):
        super(PlacementFinder, self).__init__(
            "find_placement",
            None
        )

        rospy.loginfo(
            "%s: Waiting for C++ placement checking service..." % self.__class__.__name__
        )
        rospy.wait_for_service("/CheckPlacement")
        self.placement_checking_service = rospy.ServiceProxy( "/CheckPlacement", CheckPlacement)
        self.placement_finding_service = rospy.Service("find_placement_around", FindPlacement, self.find_placement)
        rospy.loginfo("%s: Initialised. Ready for clients." % self.__class__.__name__)

    def find_placement(self, goal_msg):
        resp = FindPlacementResponse()
        foundFlag = False

        goal_tf = goal_msg.goal_tf
        dims = goal_msg.dims
        maxHeight, radius, candidateNum = goal_msg.maxHeight, goal_msg.radius, goal_msg.candidateNum

        (origin_tf, _) = self.lookup_transform(self.MAP_FRAME, goal_tf)
        origin_x, origin_y, origin_z = origin_tf.translation.x, origin_tf.translation.y, origin_tf.translation.z

        (base_tf, _) = self.lookup_transform(self.MAP_FRAME, self.BASE_FRAME)
        angle_to_base = math.atan2(base_tf.translation.y - origin_tf.translation.y,
                                   base_tf.translation.x - origin_tf.translation.x)

        # Define Bounding Box
        assert len(dims) == 1 or len(dims) == 3, "Dimensions of the bounding box not set correctly"
        if len(dims) == 1:
            dims = dims * 3

        angleStep = 2 * math.pi / candidateNum
        idx = list(range(candidateNum // 2 - candidateNum, candidateNum // 2))
        idx.sort(key=abs)
        for ii in idx:
            angle_to_candidate = angle_to_base + ii * angleStep
            candidatePos = Point(origin_x + math.cos(angle_to_candidate) * radius,
                                 origin_y + math.sin(angle_to_candidate) * radius,
                                 origin_z)

            dimsInput = Point(*dims)
            checkerResp = self.placement_checking_service(candidatePos, dimsInput, maxHeight)

            if (checkerResp.isAvailable and checkerResp.isSupported):
                if not foundFlag:
                    resp.position = (candidatePos.x, candidatePos.y, candidatePos.z)
                    foundFlag = True

                pose = geometry.Pose(
                    geometry.Vector3(
                        candidatePos.x, candidatePos.y, candidatePos.z
                    ),
                    geometry.Quaternion(0.5, 0.5, 0.5, 0.5), # TODO: Use a better Quaternion
                )
                self.publish_goal_pose_tf(pose, self.MAP_FRAME, "placement_candidate" + str(ii))

        return resp

if __name__ == "__main__":
    rospy.init_node("find_placement_server_node")
    server = PlacementFinder()
    rospy.spin()