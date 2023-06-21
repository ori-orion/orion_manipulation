#! /usr/bin/env python3

import math

import hsrb_interface.geometry as geometry

import rospy
import tf2_ros
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
        
        self.static_broadcaster = tf2_ros.StaticTransformBroadcaster()

        rospy.loginfo("%s: Initialised. Ready for clients." % self.__class__.__name__)

    def find_placement(self, goal_msg):
        print("START find_placement");
        resp = FindPlacementResponse()

        goal_tf = goal_msg.goal_tf
        goal_pos = goal_msg.goal_pos
        dims = goal_msg.dims
        maxHeight, radius, candidateNum, z_shift = goal_msg.maxHeight, goal_msg.radius, goal_msg.candidateNum, goal_msg.z_shift

        if goal_tf != "":
            (origin_tf, _) = self.lookup_transform(self.MAP_FRAME, goal_tf)
            origin_x, origin_y, origin_z = origin_tf.translation.x, origin_tf.translation.y, origin_tf.translation.z
        else:
            assert len(goal_pos) == 3, "Dimensions of the goal TF position not set correctly"
            origin_x, origin_y, origin_z = goal_pos

        (base_tf, _) = self.lookup_transform(self.MAP_FRAME, self.BASE_FRAME)
        angle_to_base = math.atan2(base_tf.translation.y - origin_y,
                                   base_tf.translation.x - origin_x)

        # Define Bounding Box
        assert len(dims) == 1 or len(dims) == 3, "Dimensions of the bounding box not set correctly"
        if len(dims) == 1:
            dims = dims * 3

        angleStep = 2 * math.pi / candidateNum
        idx = list(range(candidateNum // 2 - candidateNum, candidateNum // 2))
        idx.sort(key=abs)
        for ii in idx:
            print("\tChecking candidate {0}".format(ii));
            angle_to_candidate = angle_to_base + ii * angleStep
            candidatePos = Point(origin_x + math.cos(angle_to_candidate) * radius,
                                 origin_y + math.sin(angle_to_candidate) * radius,
                                 origin_z + z_shift)

            dimsInput = Point(*dims)
            checkerResp = self.placement_checking_service(candidatePos, dimsInput, maxHeight)

            print("Is available:", checkerResp.isAvailable, "Is supported:", checkerResp.isSupported);

            if (checkerResp.isAvailable and checkerResp.isSupported):
                tf_name = "placement_candidate"+str(ii);
                
                resp.position = (candidatePos.x, candidatePos.y, candidatePos.z)
                resp.best_tf = tf_name;
                print("\tGoal found");
                
                pose = geometry.Pose(
                    geometry.Vector3(
                        candidatePos.x, candidatePos.y, candidatePos.z
                    ),
                    geometry.Quaternion(0, 0, 0, 1), # TODO: Use a better Quaternion
                )
                self.publish_goal_pose_tf(pose, self.MAP_FRAME, tf_name)
                
                break

        print("Finished");

        return resp

    def publish_tf(self, transform, source_frame_id, child_frame_id):
        """
        Convenience function to publish a transform.
        Args:
            transform: geometry_msgs Transform type, from source_frame to child_frame
            source_frame_id: name of source frame
            child_frame_id: name of child frame
        
        Overrides the function in ManipulationHeader
        """
        rospy.loginfo("Using the overridden publish_tf");
        t = TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = source_frame_id
        t.child_frame_id = child_frame_id
        t.transform = transform
        self.static_broadcaster.sendTransform(t)

if __name__ == "__main__":
    rospy.init_node("find_placement_server_node")
    server = PlacementFinder()
    rospy.spin()