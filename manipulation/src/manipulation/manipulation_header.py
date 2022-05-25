from std_srvs.srv import Empty
from manipulation.srv import GetReconstruction, GetReconstructionResponse
from geometry_msgs.msg import Vector3, Pose, Quaternion
import rospy

from hsrb_interface.collision_world import CollisionWorld
import hsrb_interface.geometry as geometry

from hsrb_interface import robot as _robot
_robot.enable_interactive()


class CollisionMapper:
    """
    This class is an interface to the robot's global collision world - the only collision
    world we are allowed to have...
    """

    def __init__(self, robot):
        self.robot = robot
        self.global_collision_world = self.robot.try_get('global_collision_world')

        print(f"Waiting for octomap reset service")
        rospy.wait_for_service("/octomap_server/reset")
        self.reset_service = rospy.ServiceProxy("/octomap_server/reset", Empty)

        print(f"Waiting for reconstruction service...")
        rospy.wait_for_service("/GetReconstruction")
        self.reconstruction_service = rospy.ServiceProxy("/GetReconstruction", GetReconstruction)

    def reset_collision_map(self):
        try:
            self.reset_service()
        except rospy.ServiceException as e:
            print(f"Service call failed: {e}")

        # Clear everything in global collision map
        self.global_collision_world.remove_all()

    def get_converted_octomap(self, external_bb, crop_bbs, stl_path):
        try:
            resp = self.reconstruction_service(external_bb, crop_bbs, stl_path)
            return resp.flag

        except rospy.ServiceException as e:
            print(f"Service call failed: {e}")

    def build_collision_world(self, external_bounding_box, crop_bounding_boxes=None, stl_path = "/tmp/tmp.stl"):
        crop_bbs = [] if crop_bounding_boxes is None else crop_bounding_boxes

        # Reset reconstruction
        self.reset_collision_map()

        # Wait for map to populate
        # TODO confirm that Octomap is actually building the map during this sleep
        rospy.sleep(3)

        # Get and return collision map generated over last 3s
        flag = self.get_converted_octomap(external_bounding_box, crop_bbs, stl_path)

        # Add the collision map (from octomap) to the global collision world
        if flag:
            self.add_map_to_global_collision_world(stl_path)

    def add_map_to_global_collision_world(self, stl_path):
        self.global_collision_world.add_mesh(stl_path, frame_id="map", timeout=0.0)

        return


# import numpy as np
# import tf

# def check_for_object(object_tf):
#     tf_listener = tf.TransformListener()
#     foundMarker = False
#     while not foundMarker:
#         all_frames = tf_listener.getFrameStrings()
#         foundMarker = object_tf in all_frames
#
#
# def get_similar_tf(tf_frame):
#     tf_listener = tf.TransformListener()
#     rospy.sleep(3)
#     all_frames = tf_listener.getFrameStrings()
#     print(all_frames)
#     for object_tf in all_frames:
#         print '----------------'
#         print object_tf
#         print object_tf.split('-')[0]
#         print '----------------'
#         if tf_frame.split('_')[-1] in object_tf.split('-')[0]:
#             return object_tf
#
#
# def get_object_pose(object_tf):
#     tf_listener = tf.TransformListener()
#     foundTrans = False
#     while not foundTrans:
#         try:
#             t = tf_listener.getLatestCommonTime("/map", object_tf)
#             (trans, rot) = tf_listener.lookupTransform('/map', object_tf, t)
#             foundTrans = True
#         except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
#             continue
#
#     return np.array([trans[0], trans[1], trans[2]])
