from std_srvs.srv import Empty
import rospy


class CollisionMapper:
    def __init__(self, robot):
        self.robot = robot

    def reset_collision_map_build(self):
        rospy.wait_for_service('/tmc_reconstruction/system/reset')
        try:
            reset_service = rospy.ServiceProxy('/tmc_reconstruction/system/reset', Empty)
            reset_service()
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e

    def start_collision_map_build(self):
        rospy.wait_for_service('/tmc_reconstruction/system/start')
        try:
            start_service = rospy.ServiceProxy('/tmc_reconstruction/system/start', Empty)
            start_service()
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e

    def stop_collision_map_build(self):
        rospy.wait_for_service('/tmc_reconstruction/system/stop')
        try:
            stop_service = rospy.ServiceProxy('/tmc_reconstruction/system/stop', Empty)
            stop_service()
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e

    def remove_all_collision_objects(self):
        collision_world = self.robot.try_get('global_collision_world')
        collision_world.remove_all()

    def get_collision_map(self):

        # Clear objects
        self.remove_all_collision_objects()

        # Reset reconstruction
        self.reset_collision_map_build()

        # Clear objects again to be sure
        self.remove_all_collision_objects()

        # Start reconstruction
        self.start_collision_map_build()

        # Wait for map to populate
        rospy.sleep(3)

        # Stop reconstruction
        self.stop_collision_map_build()
        rospy.sleep(2)

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


