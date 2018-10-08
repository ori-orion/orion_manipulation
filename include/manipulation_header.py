from tmc_manipulation_msgs.msg import CollisionObject
import numpy as np
from std_srvs.srv import Empty
import rospy
import hsrb_interface
import tf


def checkForObject(object_tf):
    tf_listener = tf.TransformListener()
    foundMarker = False
    while not foundMarker:
        all_frames = tf_listener.getFrameStrings()
        print all_frames
        foundMarker = object_tf in all_frames
    print "Found the object."


def get_Object_Pose(object_tf):
    print "Checking object is in sight..."
    checkForObject(object_tf)

    print "Checking object is in sight..."
    tf_listener = tf.TransformListener()
    foundTrans = False
    while not foundTrans:
        try:
            (trans, rot) = tf_listener.lookupTransform('/map', object_tf, rospy.Time(0))
            foundTrans = True
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

    return np.array([trans[0], trans[1], trans[2]])


def resetCollisionMapBuild():
    rospy.wait_for_service('/tmc_reconstruction/system/reset')
    try:
        reset_service = rospy.ServiceProxy('/tmc_reconstruction/system/reset', Empty)
        reset_service()
        print "tmc_reconstruction reset."
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e

def startCollisionMapBuild():
    rospy.wait_for_service('/tmc_reconstruction/system/start')
    try:
        start_service = rospy.ServiceProxy('/tmc_reconstruction/system/start', Empty)
        start_service()
        print "tmc_reconstruction started."
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e


def stopCollisionMapBuild():
    rospy.wait_for_service('/tmc_reconstruction/system/stop')
    try:
        stop_service = rospy.ServiceProxy('/tmc_reconstruction/system/stop', Empty)
        stop_service()
        print "tmc_reconstruction stopped."
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e


def removeAllCollisionObjects(robot):
    collision_world = robot.try_get('global_collision_world')
    collision_world.remove_all()

def getCollisionMap(robot):

    # Clear objects
    removeAllCollisionObjects(robot)

    # Reset reconstruction
    resetCollisionMapBuild()

    # Clear objects again to be sure
    removeAllCollisionObjects(robot)

    # Start reconstruction
    startCollisionMapBuild()

    # Wait for map to populate
    sleep_time = 3
    print "Waiting for " + str(sleep_time) + " seconds..."
    rospy.sleep(sleep_time)

    # Stop reconstruction
    stopCollisionMapBuild()