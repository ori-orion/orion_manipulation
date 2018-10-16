import numpy as np
from std_srvs.srv import Empty
import rospy
import tf


def check_for_object(object_tf):
    tf_listener = tf.TransformListener()
    foundMarker = False
    while not foundMarker:
        all_frames = tf_listener.getFrameStrings()
        foundMarker = object_tf in all_frames

def get_similar_tf(tf_frame):
   
    tf_listener = tf.TransformListener()
    all_frames = tf_listener.getFrameStrings()
    foundMarker = False
    for object_tf in all_frames:
        if tf_frame == object_tf.split('-')[0]:
		return tf_frame	 

def get_object_pose(object_tf):
    #print "Checking object is in sight..."
    #check_for_object(object_tf)

    tf_listener = tf.TransformListener()
    foundTrans = False
    while not foundTrans:
        try:
	    t = tf_listener.getLatestCommonTime("/map", object_tf)
	    (trans, rot) = tf_listener.lookupTransform('/map', object_tf, t)
            #(trans, rot) = tf_listener.lookupTransform('/map', object_tf, rospy.Time(0))
            foundTrans = True
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

    return np.array([trans[0], trans[1], trans[2]])


def reset_collision_map_build():
    rospy.wait_for_service('/tmc_reconstruction/system/reset')
    try:
        reset_service = rospy.ServiceProxy('/tmc_reconstruction/system/reset', Empty)
        reset_service()
        #print "tmc_reconstruction reset."
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e

def start_collision_map_build():
    rospy.wait_for_service('/tmc_reconstruction/system/start')
    try:
        start_service = rospy.ServiceProxy('/tmc_reconstruction/system/start', Empty)
        start_service()
        #print "tmc_reconstruction started."
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e


def stop_collision_map_build():
    rospy.wait_for_service('/tmc_reconstruction/system/stop')
    try:
        stop_service = rospy.ServiceProxy('/tmc_reconstruction/system/stop', Empty)
        stop_service()
        #print "tmc_reconstruction stopped."
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e


def remove_all_collision_objects(robot):
    collision_world = robot.try_get('global_collision_world')
    collision_world.remove_all()

def get_collision_map(robot):

    # Clear objects
    remove_all_collision_objects(robot)

    # Reset reconstruction
    reset_collision_map_build()

    # Clear objects again to be sure
    remove_all_collision_objects(robot)

    # Start reconstruction
    start_collision_map_build()

    # Wait for map to populate
    sleep_time = 3
    rospy.sleep(sleep_time)

    # Stop reconstruction
    stop_collision_map_build()
    rospy.sleep(1)
