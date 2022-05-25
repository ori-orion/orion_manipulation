#! /usr/bin/env python3
""" Common manipulation action server functionality.
"""

import rospy
import actionlib
import math
import tf2_ros
import hsrb_interface
import hsrb_interface.geometry as geometry
from std_srvs.srv import Empty

from manipulation.srv import GetReconstruction

from hsrb_interface import robot as _robot
_robot.enable_interactive()


class ManipulationAction(object):

    # Robot default parameters
    DEFAULT_BODY_PLANNING_TIMEOUT = 20.0  # Robot planning timeout - default is 10s
    DEFAULT_BODY_TF_TIMEOUT = 10.0  # Robot TF timeout - default is 5s

    # Frame name constants
    HAND_FRAME = "hand_palm_link"
    SUCKER_FRAME = "hand_l_finger_vacuum_frame"
    RGBD_CAMERA_FRAME = "head_rgbd_sensor_rgb_frame"
    ODOM_FRAME = "odom"
    MAP_FRAME = "map"
    BASE_FRAME = "base_link"

    def __init__(self, action_name, action_msg_type, use_collision_map=True, tts_narrate=True):
        """
        Base class for manipulation actions.
        Args:
            action_name: The name of the action server
            action_msg_type: ROS message type for the action
            use_collision_map: Whether to use dynamic collision mapping to avoid obstacles
            tts_narrate: Whether to narrate what the robot is doing out loud with
                text-to-speech

        """

        self._action_name = action_name
        self._action_msg_type = action_msg_type
        self.use_collision_map = use_collision_map

        # Preparation for using the robot functions
        self.robot = hsrb_interface.Robot()
        self.whole_body = self.robot.try_get('whole_body')
        self.omni_base = self.robot.try_get('omni_base')
        self.collision_world = None
        self.gripper = self.robot.try_get('gripper')
        self.whole_body.end_effector_frame = self.HAND_FRAME
        self.whole_body.looking_hand_constraint = True
        self.tts_narrate = tts_narrate
        if self.tts_narrate:
            self.tts = self.robot.try_get('default_tts')
            self.tts.language = self.tts.ENGLISH

        self.whole_body.planning_timeout = self.DEFAULT_BODY_PLANNING_TIMEOUT
        self.whole_body.tf_timeout = self.DEFAULT_BODY_TF_TIMEOUT

        self.collision_mapper = CollisionMapper(self.robot) if use_collision_map else None

        # TF, defaults to buffering 10 seconds of transforms
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer)

        # All manipulation actions can publish a "goal" tf
        self.goal_pose_br = tf2_ros.TransformBroadcaster()

        self._as = actionlib.SimpleActionServer(self._action_name, self._action_msg_type,
                                                execute_cb=self.execute_cb, auto_start=False)
        self._as.start()
        rospy.loginfo("%s: Action server started." % self._action_name)

    def execute_cb(self, goal_msg):
        """
        Do any bookkeeping here e.g. store pre-action map pose.
        """
        rospy.loginfo("%s: Received goal: %s" % (self._action_name, goal_msg))
        return self._execute_cb(goal_msg)

    def abandon_action(self):
        """
        Abandon manipulation action: move robot to go position and abort action server.
        """
        rospy.loginfo('%s: Aborted. Moving to go and exiting.' % self._action_name)
        self.whole_body.move_to_go()
        self._as.set_aborted()

    def preempt_action(self):
        """
        Preempt manipulation action: move robot to go position and preempt action server.
        """
        rospy.loginfo('%s: Preempted. Moving to go and exiting.' % self._action_name)
        self.tts_say("I was preempted. Moving to go.")
        self.whole_body.move_to_go()
        self._as.set_preempted()

    def publish_goal_pose_tf(self, p):
        """
        Publish a tf to the goal pose, in odom frame
        Args:
            p:mgoal pose
        """
        self.goal_pose_br.sendTransform((p.pos.x, p.pos.y, p.pos.z),
                                        (p.ori.x, p.ori.y, p.ori.z, p.ori.w),
                                        rospy.Time.now(),
                                        'goal_pose',
                                        'odom')

    def lookup_transform(self, source, dest):
        """
        Lookup a transfrom and its timestamp.
        Args:
            source: name of source frame
            dest: name of dest frame
        Returns: (transform, time) where transform is a geometry_msgs Transform and time
                  is a rostime.Time
        """
        try:
            trans_stamped = self._tf_buffer.lookup_transform(source, dest, rospy.Time())
            return trans_stamped.transform, trans_stamped.header.stamp

        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ):
            return None, None

    def calc_transform_distance(self, t):
        """
        Calculate the L2 distance represented by a transform.
        Args:
            t: transform
        """
        return math.sqrt(math.pow(t[0], 2) + math.pow(t[1], 2) + math.pow(t[2], 2))

    def tts_say(self, string_to_say):
        """
        Say something via text-to-speech, if text-to-speech narration is enabled.
        Args:
            string_to_say: string to be said out loud by the robot.
        """
        if self.tts_narrate:
            self.tts.say(string_to_say)


class CollisionMapper:
    """
    This class is an interface to collision mapping on the robot.
    This is not intended to be used asynchronously, but avoids being broken by
    asynchronous by nodes to the robot's collision map.
    """

    def __init__(self, robot):
        self.robot = robot
        self.global_collision_world = self.robot.try_get("global_collision_world")

        rospy.loginfo(
            "%s: Waiting for octomap reset service..." % self.__class__.__name__
        )
        rospy.wait_for_service("/octomap_server/reset")
        self.reset_service = rospy.ServiceProxy("/octomap_server/reset", Empty)

        rospy.loginfo(
            "%s: Waiting for reconstruction service..." % self.__class__.__name__
        )
        rospy.wait_for_service("/GetReconstruction")
        self.reconstruction_service = rospy.ServiceProxy(
            "/GetReconstruction", GetReconstruction
        )

    def reset_collision_map(self):
        try:
            self.reset_service()
        except rospy.ServiceException as e:
            print(f"Service call failed: {e}")

        # Clear everything in global collision map
        self.global_collision_world.remove_all()

    def get_converted_octomap(self, external_bb, crop_bbs):
        try:
            resp = self.reconstruction_service(external_bb, crop_bbs)
            return resp.resp

        except rospy.ServiceException as e:
            print(f"Service call failed: {e}")

    def build_collision_world(self, external_bounding_box, crop_bounding_boxes=None):

        crop_bbs = [] if crop_bounding_boxes is None else crop_bounding_boxes

        # Reset reconstruction
        self.reset_collision_map()

        # Wait for map to populate
        # TODO confirm that Octomap is actually building the map during this sleep
        rospy.sleep(3)

        # Get and return collision map generated over last 3s
        tmc_collision_map = self.get_converted_octomap(external_bounding_box, crop_bbs)

        # Add the collision map (from octomap) to the global collision world
        self.add_map_to_global_collision_world(tmc_collision_map)

    def add_map_to_global_collision_world(self, collision_map_msg):

        i = 0
        for box in collision_map_msg.boxes:
            print("Processing" + str(i) + ", pos =" + str(box.center.x) + ", " + str(box.center.y) + ", " + str(box.center.z))
            i += 1
            # pos = Vector3(box.center.x, box.center.y, box.center.z)
            size_x = box.extents.x
            size_y = box.extents.y
            size_z = box.extents.z
            # Other variables in msg:
            # box.axis.x = 1.0;
            # box.axis.y = 0.0;
            # box.axis.z = 0.0;
            # box.angle = 0.0;

            self.global_collision_world.add_box(
                x=size_x,
                y=size_y,
                z=size_z,
                pose=geometry.pose(x=box.center.x, y=box.center.y, z=box.center.z),
                frame_id="map",
                timeout=0.0,
            )

        return
