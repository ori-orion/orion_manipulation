from std_srvs.srv import Empty
import rospy


class CollisionMapper:
    def __init__(self, robot):
        self.robot = robot

    def reset_collision_map(self):
        rospy.wait_for_service('/octomap_server/reset')
        try:
            reset_service = rospy.ServiceProxy('/octomap_server/reset', Empty)
            reset_service()
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e

    def get_converted_octomap(self):
        rospy.wait_for_service('/GetReconstruction')
        try:
            map_service = rospy.ServiceProxy('/GetReconstruction', Empty)
            resp = map_service()
            return resp.resp
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e

    def get_collision_map(self):

        # Reset reconstruction
        self.reset_collision_map()

        # Wait for map to populate
        rospy.sleep(3)

        # Get and return collision map generated over last 3s
        return self.get_converted_octomap()

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


