#! /usr/bin/env python

import numpy as np
import tf.transformations as T
import rospy
import tf

# Select a grasp for the robot to execute.
from gpd.msg import *



def callback(msg):
    br = tf.TransformBroadcaster()
    grasps = msg.grasps

    if len(grasps) > 0:
        rospy.loginfo('Received %d grasps.', len(grasps))

        grasp = grasps[0] # grasps are sorted in descending order by score
        print 'Selected grasp with score:', grasp.score

        # This gives the approach point correctly
        bottom = np.array([grasp.bottom.x, grasp.bottom.y, grasp.bottom.z])
        approach = np.array([grasp.approach.x, grasp.approach.y, grasp.approach.z ])
        binormal = np.array([grasp.binormal.x, grasp.binormal.y, grasp.binormal.z])
        hand_outer_diameter = 0.12
        hw = 0.5*hand_outer_diameter
        finger_width = 0.01
        left_bottom = bottom - (hw - 0.5 * finger_width) * binormal
        right_bottom = bottom + (hw - 0.5 * finger_width) * binormal
        base_center = left_bottom + 0.5 * (right_bottom - left_bottom) - 0.01 * approach
        approach_center = base_center - 0.06 * approach

        approach_4 = np.array([grasp.approach.x, grasp.approach.y, grasp.approach.z , approach_center[0]])
        binormal_4 = np.array([grasp.binormal.x, grasp.binormal.y, grasp.binormal.z, approach_center[1]])
        axis_4 = np.array([grasp.axis.x, grasp.axis.y, grasp.axis.z, approach_center[2]])

        R = np.array([approach_4, binormal_4, axis_4, [0, 0, 0, 1]])
        q = T.quaternion_from_matrix(R)

        # NOTE THIS NEEDS THE QUATERNION CONJUGATE
        br.sendTransform((approach_center[0], approach_center[1], approach_center[2]),
                         T.quaternion_conjugate(q),
                             rospy.Time.now(),
                             'goal_pose',
                             'head_rgbd_sensor_rgb_frame')

rospy.init_node('select_grasp')

# Subscribe to the ROS topic that contains the grasps.
grasps_sub = rospy.Subscriber('/detect_grasps/clustered_grasps', GraspConfigList, callback)

rospy.spin()