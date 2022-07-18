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
from geometry_msgs.msg import TransformStamped

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

    # Whether to finally return to the map position the manipulation action was called at
    RETURN_TO_START_AFTER_ACTION = True

    # Kinematic parameters
    MAX_HEIGHT_ARM_LIFT_JOINT = 0.69
    # every 2cm the arm lift join lifts, the RGBD camera moves up by 1cm
    H_ARM_RGBD_SCALE_FACTOR = 2
    RGBD_MIN_HEIGHT = 0.967
    RGBD_MAX_HEIGHT = 1.312
    MIN_ANGLE_ARM_FLEX_HEIGHT = -2.6

    # Parameters estimated from RViz / robot
    # How far the arm swings when switching it from 0 to MIN_ANGLE_ARM_FLEX_HEIGHT radians
    ARM_SWING_DIST = 0.3  # metres
    MAX_HEIGHT_PLANE_VISIBLE = 3.0  # metres
    # Should aim to look down from about 10cm above
    LOOK_ANGLE_OFFSET = 0.1  # metres
    # How far up can ARM LIGHT JOINT go without the hand blocking the RGBD camera?
    MIN_HEIGHT_ARM_LIFT_JOINT_NO_HAND_OCCLUSION = 0.25  # metres

    def __init__(self, action_name, action_msg_type, use_collision_map=True, tts_narrate=True, prevent_motion=False):
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

        # TF defaults to buffering 10 seconds of transforms. Choose 60 second buffer.
        self._tf_buffer = tf2_ros.Buffer(rospy.Duration.from_sec(60.0))
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

        action_start_pose = self.omni_base.pose
        rospy.loginfo("%s: Stored action start pose: %s"
                      % (self._action_name, action_start_pose))

        retval = self._execute_cb(goal_msg)

        if self.RETURN_TO_START_AFTER_ACTION:
            rospy.loginfo("%s: Returning to start pose: %s"
                          % (self._action_name, action_start_pose))
            self.omni_base.go_abs(
                x=action_start_pose[0],
                y=action_start_pose[1],
                yaw=action_start_pose[2],
                timeout=10.0)

        return retval

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
            p: goal pose
        """
        t = TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = "odom"
        t.child_frame_id = "goal_pose"
        t.transform.translation.x = p.pos.x
        t.transform.translation.y = p.pos.y
        t.transform.translation.z = p.pos.z
        t.transform.rotation.x = p.ori.x
        t.transform.rotation.y = p.ori.y
        t.transform.rotation.z = p.ori.z
        t.transform.rotation.w = p.ori.w

        self.goal_pose_br.sendTransform(t)

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
        return math.sqrt(math.pow(t.translation.x, 2) + math.pow(t.translation.y, 2) + math.pow(t.translation.z, 2))

    def tts_say(self, string_to_say):
        """
        Say something via text-to-speech, if text-to-speech narration is enabled.
        Args:
            string_to_say: string to be said out loud by the robot.
        """
        if self.tts_narrate:
            self.tts.say(string_to_say)

    # Common kinematic functionality

    def move_arm_down(self, arm_lift_joint_height, back_away=True):
        """
        Move backwards and flip arm down.
        """
        if back_away:
            self.omni_base.go_pose(
                geometry.pose(x=-self.ARM_SWING_DIST), 10.0, ref_frame_id=self.BASE_FRAME
            )

        if arm_lift_joint_height < self.MIN_HEIGHT_ARM_LIFT_JOINT_NO_HAND_OCCLUSION:
            rospy.logerr(
                "%s: Arm lift joint height too low to move arm down" % self._action_name
            )
            return

        self.whole_body.move_to_joint_positions(
            {'arm_lift_joint': arm_lift_joint_height,
             'arm_flex_joint': self.MIN_ANGLE_ARM_FLEX_HEIGHT}
        )

        if back_away:
            self.omni_base.go_pose(
                geometry.pose(x=self.ARM_SWING_DIST), 10.0, ref_frame_id=self.BASE_FRAME
            )

    def look_at_object(self, object_tf, rotate_to_face=True):
        """
        Get a "best-view" look at the object at the specified tf.
        Args:
            object_transform: tf
            rotate_to_face: whether to rotate whole robot body to face object
        Returns: success, true or false
        """
        (trans, _) = self.lookup_transform(self.BASE_FRAME, object_tf)

        if trans is None:
            return False

        if rotate_to_face:
            yaw = math.atan2(trans.translation.y, trans.translation.x)
            self.omni_base.go_rel(0.0, 0.0, yaw, 10.0)

        object_height = trans.translation.z
        self.move_camera_to_height(object_height + self.LOOK_ANGLE_OFFSET)

        self.whole_body.gaze_point(point=geometry.Vector3(0, 0, 0), ref_frame_id=object_tf)

        return True

    def move_camera_to_height(self, h):
        """
        Move the RGBD camera to a specified height. Assumes start state is go position.
        Args:
            h: height in metres
        """
        if h <= self.RGBD_MIN_HEIGHT:
            return

        arm_joint_height = (h - self.RGBD_MIN_HEIGHT) * self.H_ARM_RGBD_SCALE_FACTOR

        if arm_joint_height > self.MAX_HEIGHT_ARM_LIFT_JOINT:
            rospy.logwarn(
                "%s: Can't get RGBD camera high enough to look down at goal. Target height %f m"
                % (self._action_name, h)
            )
            arm_joint_height = self.MAX_HEIGHT_ARM_LIFT_JOINT

        if arm_joint_height < self.MIN_HEIGHT_ARM_LIFT_JOINT_NO_HAND_OCCLUSION:
            self.whole_body.move_to_joint_positions(
                {'arm_lift_joint': arm_joint_height}
            )
        else:
            self.move_arm_down(arm_lift_joint_height=arm_joint_height)


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
        """
        Clears all objects from the HSR's collision map and the octomap server node's map.
        """
        try:
            self.reset_service()
        except rospy.ServiceException as e:
            print(f"Service call failed: {e}")

        # Clear everything in global collision map
        self.global_collision_world.remove_all()

    def get_converted_octomap(self, external_bb, crop_bbs, stl_path):
        """
        Requests the octomap_to_reconstruction node to build an STL mesh from the octomap
        produced by octomap_server.
        Args:
            external_bb: external BoundingBox that bounds the STL mesh to be created
            crop_bbs: list of BoundingBox to crop out of the map (e.g. covering an object)
            stl_path: path to save the STL mesh file to
        Returns: success flag returned from octomap_to_reconstruction node
        """
        try:
            resp = self.reconstruction_service(external_bb, crop_bbs, stl_path)
            return resp.flag

        except rospy.ServiceException as e:
            print(f"Service call failed: {e}")

    def build_collision_world(self, external_bounding_box, crop_bounding_boxes=None):
        """
        Add the robot's 3D occupancy map of a specified area to the HSR collision world.
        Args:
            external_bb: external BoundingBox that bounds the STL mesh to be created
            crop_bbs: list of BoundingBox to crop out of the map (e.g. covering an object)
        """

        # STL path to save to
        timestamp_str = str(rospy.get_time()).replace(".", "-")
        stl_path = "/tmp/collision_mesh_" + timestamp_str + ".stl"

        crop_bbs = [] if crop_bounding_boxes is None else crop_bounding_boxes

        # Reset reconstruction
        self.reset_collision_map()

        # Wait for map to populate
        # TODO confirm that Octomap server is actually building the map during this sleep
        rospy.sleep(3)

        # Get and return collision map generated over last 3s
        flag = self.get_converted_octomap(external_bounding_box, crop_bbs, stl_path)

        # Add the collision map (from octomap) to the global collision world
        if flag:
            self.add_map_to_global_collision_world(stl_path)
        else:
            rospy.logwarn(
                "%s: Octomap reconstruction node returned an empty collision mesh."
                % (self.__class__.__name__)
            )

        return self.global_collision_world

    def add_map_to_global_collision_world(self, stl_path):
        """
        Add an STL mesh to the HSR's global collision world.
        """
        self.global_collision_world.add_mesh(stl_path, frame_id="map", timeout=0.0)
        return
