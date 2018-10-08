from src.include.manipulation_header import *
from tmc_manipulation_msgs.msg import CollisionObject
import numpy as np
import rospy
import hsrb_interface
import hsrb_interface.geometry as geometry
import tf

def callback(msg):
    # Get the message
    message = msg

    # Find which boxes to removes
    inds_to_remove = []
    for i in range(len(message.poses)):
        pose = message.poses[i]
        if(pose.position.x <= upper_bounds[0] and pose.position.x >= lower_bounds[0] and
           pose.position.y <= upper_bounds[1] and pose.position.y >= lower_bounds[1] and
           pose.position.z <= upper_bounds[2] and pose.position.z >= lower_bounds[2]):
                inds_to_remove.append(i)

    # Remove the boxes
    for index in sorted(inds_to_remove, reverse=True):
        del message.poses[index], message.shapes[index]

    # Publish the filtered message
    pub.publish(message)
    print("Message published")

def main():
    global upper_bounds, lower_bounds, pub

    # Preparation for using the robot functions
    robot = hsrb_interface.Robot()
    whole_body = robot.try_get('whole_body')
    omni_base = robot.try_get('omni_base')
    collision_world = robot.try_get('global_collision_world')
    gripper = robot.try_get('gripper')

    _HAND_TF = 'hand_palm_link'
    _GRASP_FORCE = 0.2
    _OBJECT_TF = 'ar_marker/201'

    # Put bounds about the tf frame of object to remove from map
    exclusion_bounds = np.array([0.07, 0.07, 0.07])

    # Set the grasp poses
    pregrasp_pose = geometry.pose(z=-0.05, ek=-1.57)
    grasp_pose = geometry.pose(z=0.02)

    # Get in a head up position to see the marker
    whole_body.move_to_go()

    # Open gripper
    gripper.command(1.2)

    # Follow the gripper
    whole_body.looking_hand_constraint = True

    # Set collision map
    getCollisionMap(robot)

    # Get the object pose to subtract from collision map
    goal_object_pose = get_Object_Pose(_OBJECT_TF)
    upper_bounds = goal_object_pose + exclusion_bounds
    lower_bounds = goal_object_pose - exclusion_bounds
    print("Bounds have been set")

    # Remove object from map and publish to correct topic
    pub = rospy.Publisher('known_object', CollisionObject, queue_size=1)
    rospy.Subscriber("known_object_pre_filter", CollisionObject, callback)
    rospy.sleep(2)

    # Set up listener to find the bottle
    checkForObject(_OBJECT_TF)

    # Turn on collision checking
    whole_body.collision_world = collision_world

    print("Moving to object...")
    whole_body.move_end_effector_pose(pregrasp_pose, _OBJECT_TF)

    # Turn off collision checking to get close and grasp
    whole_body.collision_world = None
    whole_body.move_end_effector_pose(grasp_pose, 'hand_palm_link')

    # Specify the force to grasp
    gripper.apply_force(_GRASP_FORCE)
    rospy.sleep(2.0)
    print("Object grasped.")

    # Turn collision checking back on
    whole_body.collision_world = collision_world
    rospy.sleep(0.5)

    # Now return to moving position
    print("Try to move to go.")
    whole_body.move_to_go()

    print("Finished")

if __name__ == "__main__":
    main()
