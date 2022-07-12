#! /usr/bin/env python3
__author__ = "Mark Finean"
__email__ = "mfinean@robots.ox.ac.uk"

import rospy
import tf
import actionlib
from orion_actions.msg import *
from geometry_msgs.msg import PoseStamped

def put_object_on_surface_client(goal_tf):
    client = actionlib.SimpleActionClient('put_object_on_surface', PutObjectOnSurfaceAction)

    print("Waiting for server")
    client.wait_for_server()
    print("Finished waiting for server")

    # Creates a goal to send to the action server.
    goal_msg = PutObjectOnSurfaceGoal(goal_tf=goal_tf)

    # Sends the goal to the action server.
    client.send_goal(goal_msg)

    # Waits for the server to finish performing the action.
    client.wait_for_result()

    # Prints out the result of executing the action
    return client.get_result()


def publish_frame(tf_name, position, ref_frame = "head_rgbd_sensor_rgb_frame"):
    new_tf = PoseStamped()
    br = tf.TransformBroadcaster()

    new_tf.pose.position.x = position[0]
    new_tf.pose.position.y = position[1]
    new_tf.pose.position.z = position[2]
    i = 0
    iterations = 1/0.05
    while not rospy.is_shutdown() and i < iterations:
        br.sendTransform((new_tf.pose.position.x, new_tf.pose.position.y, new_tf.pose.position.z),
                                    (0,0,0,1),
                                    rospy.Time.now(),
                                    tf_name,
                                    ref_frame)
        rospy.sleep(0.05)
        i += 1 

    rospy.loginfo("Published surface frame: %s" % tf_name)
    

if __name__ == '__main__':
    rospy.init_node('put_object_on_surface_client')
    if len(sys.argv) == 2:
        goal_tf = sys.argv[1]
    else:
        print("Failed to provide tf frame as argument to put object down. Setting to default surface goal..")
        goal_tf = "surface_goal_tf"
        # publish_frame(goal_tf,[-0.15,0.15,0.8])


    result = put_object_on_surface_client(goal_tf)
    print("Result:" + str(result.result))


