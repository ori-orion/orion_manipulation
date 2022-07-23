#!/usr/bin/env python3

import message_filters
from orion_actions.msg import Detection, DetectionArray, Label
import sys
import cv2
import numpy as np
import math
import rospy
import std_msgs.msg
from geometry_msgs.msg import Point, TransformStamped
from sensor_msgs.msg import CameraInfo, Image
from cv_bridge import CvBridge, CvBridgeError
import tf2_ros


min_acceptable_score = 0.50
# When performing non-maximum suppression, the intersection-over-union threshold defines
# the proportion of intersection a bounding box must cover before it is determined to be
# part of the same object.
iou_threshold = 0.80


class BboxPublisher(object):
    def __init__(self, image_topic, depth_topic, bbox_name="target", bbox_colour="red"):
        self.bbox_name = bbox_name
        self.bbox_colour = bbox_colour

        # Subscribers
        self.image_sub = message_filters.Subscriber(
            image_topic, Image, queue_size=1
        )  # TODO - see if changing from 100 will help for sim image backlog
        self.depth_sub = message_filters.Subscriber(depth_topic, Image)

        # synchronise subscribers
        self.subscribers = message_filters.ApproximateTimeSynchronizer(
            [self.image_sub, self.depth_sub], 1, 1.0
        )

        # Publishers
        self.image_pub = rospy.Publisher("/vision/bbox_image", Image, queue_size=10)
        self.detections_pub = rospy.Publisher(
            "/vision/bbox_detections", DetectionArray, queue_size=10
        )

        # Image calibrator
        camera_info = rospy.wait_for_message(
            "/hsrb/head_rgbd_sensor/depth_registered/camera_info", CameraInfo
        )
        self._invK = np.linalg.inv(np.array(camera_info.K).reshape(3, 3))

        # Define bridge open cv -> RosImage
        self.bridge = CvBridge()

        # Register a subscriber
        self.subscribers.registerCallback(self.callback)

        self.first_call = True
        self._br = tf2_ros.TransformBroadcaster()

    def getMeanDepth_gaussian(self, depth):
        """Ok so we want to mean over the depth image using a gaussian centred at the
        mid point
        Gausian is defined as e^{-(x/\sigma)^2}

        Now, e^{-(x/sigma)^2}|_{x=1.5, sigma=1}=0.105 which is small enough. I'll therefore set the width of
        the depth image to be 3 standard deviations. (Remember, there're going to be two distributions
        multiplied together here! so that makes the corners 0.011 times as strong as the centre of the image.)

        I'm then going to do (2D_gaussian \cdot image) / sum(2D_gaussian)
            (accounting for valid and invalid depth pixels on the way.)

        This should give a fairly good approximation for the depth.
        """

        def shiftedGaussian(x: float, shift: float, s_dev: float) -> float:
            return math.exp(-pow((x - shift) / s_dev, 2))

        width: float = depth.shape[0]
        height: float = depth.shape[1]
        x_s_dev: float = width / 3
        y_s_dev: float = height / 3
        x_shift: float = width / 2
        y_shift: float = height / 2

        # We need some record of the total amount of gaussian over the image so that we can work out
        # what to divide by.
        gaussian_sum: float = 0
        depth_sum: float = 0

        for x in range(width):
            x_gaussian = shiftedGaussian(x, x_shift, x_s_dev)
            for y in range(height):
                if depth[x, y] != 0:
                    point_multiplier: float = x_gaussian * shiftedGaussian(
                        y, y_shift, y_s_dev
                    )
                    gaussian_sum += point_multiplier
                    depth_sum += depth[x, y] * point_multiplier
                pass
            pass

        return depth_sum / gaussian_sum

    def callback(self, ros_image, depth_data):
        if not self.first_call:
            # Only run callback once
            return

        self.first_call = False

        print(
            "\n\n----------------------------------------------------------------------"
        )
        print("Left click and drag to define bounding box")

        # get images from cv bridge
        image = self.bridge.imgmsg_to_cv2(ros_image, "bgr8")
        depth = np.array(
            self.bridge.imgmsg_to_cv2(depth_data, "passthrough"), dtype=np.float32
        )

        show_img = image.copy()

        def mouse_event(event, x, y, flags, param):
            global start_x, start_y, drawing_rectangle, finished_drawing, end_x, end_y

            if event == cv2.EVENT_LBUTTONDOWN:
                start_x, start_y = x, y

            elif event == cv2.EVENT_MOUSEMOVE:
                try:
                    if start_x != -1:
                        show_img = image.copy()
                        cv2.rectangle(
                            show_img,
                            pt1=(start_x, start_y),
                            pt2=(x, y),
                            color=(0, 255, 255),
                            thickness=2,
                        )

                        cv2.imshow("image", show_img)
                except NameError:
                    pass

            elif event == cv2.EVENT_LBUTTONUP:
                drawing_rectangle = False
                finished_drawing = True
                end_x, end_y = x, y

        cv2.namedWindow("image")
        cv2.setMouseCallback("image", mouse_event)

        while True:
            cv2.imshow("image", show_img)
            cv2.waitKey(200)

            try:
                _ = end_x
                break
            except NameError:
                pass
        cv2.destroyAllWindows()

        ymin = min(start_y, end_y)
        xmin = min(start_x, end_x)
        ymax = max(start_y, end_y)
        xmax = max(start_x, end_x)

        boxes = np.array([[xmin, ymin, xmax, ymax]])
        labels = [1]
        scores = [1.0]

        print(
            "Publishing bounding box " + self.bbox_name + " at " + str(list(boxes[0]))
        )

        detections = []
        boxes_nms = []
        scores_nms = []
        labels_nms = []
        # NOTE: Start of block to be tested ------
        boxes_per_label = {}
        scores_per_label = {}
        detections_per_label = {}
        # NOTE: End of block to be tested ------

        for i in range(len(boxes)):
            box = boxes[i]
            label = labels[i]
            score = scores[i]

            # Dimensions of bounding box
            center_x = (box[0] + box[2]) / 2
            width = box[2] - box[0]
            center_y = (box[1] + box[3]) / 2
            height = box[3] - box[1]

            # Get depth
            trim_depth = depth[int(box[0]) : int(box[2]), int(box[1]) : int(box[2])]
            valid = trim_depth[np.nonzero(trim_depth)]

            # Discard any bounding boxes with zero trim_depth size
            if trim_depth.size == 0:
                continue

            # Use depth to get position, and if depth is not valid, discard bounding box
            if valid.size != 0:
                z = np.min(valid) * 1e-3
                # z = self.getMeanDepth_gaussian(trim_depth) * 1e-3;

                top_left_3d = np.array([int(box[0]), int(box[1]), 0])
                top_left_camera = np.dot(self._invK, top_left_3d) * z
                bottom_right_3d = np.array([int(box[2]), int(box[3]), 0])
                bottom_right_camera = np.dot(self._invK, bottom_right_3d) * z
                corner_to_corner = top_left_camera - bottom_right_camera
                x_size = abs(corner_to_corner[0])
                y_size = abs(corner_to_corner[1])
                z_size = (x_size + y_size) / 2.0
                size = Point(x_size, y_size, z_size)
            else:
                size = Point(0.0, 0.0, 0.0)
                print("\n")
                print("\n")
                print("Trim depth size: {}".format(trim_depth.size))
                print("\tno valid depth for object size", end="")
                print("\n")
                print("\n")

                continue

            # Find object position
            image_point = np.array([int(center_x), int(center_y), 1])
            obj = np.dot(self._invK, image_point) * z

            # create label
            label_str = self.bbox_name
            score_lbl = Label(label_str, np.float64(score))

            print("\n", label_str, end="")

            # create detection instance
            detection = Detection(
                score_lbl,
                center_x,
                center_y,
                width,
                height,
                size,
                self.bbox_colour,
                obj[0],
                obj[1],
                obj[2],
            )

            detections.append(detection)

        """ compatible with old version of keep
        """
        clean_detections = detections

        # Draw bounding boxes of cleaned detections onto image
        top_left = (boxes[0][0], boxes[0][1])
        bottom_right = (boxes[0][2], boxes[0][3])
        cv2.rectangle(image, top_left, bottom_right, (255, 0, 0), 3)
        cv2.putText(
            image,
            self.bbox_name + ": " + str(1.0),
            top_left,
            cv2.FONT_HERSHEY_COMPLEX,
            0.5,
            (0, 255, 0),
            1,
        )

        image_msg = self.bridge.cv2_to_imgmsg(image, "bgr8")

        h = std_msgs.msg.Header()
        h.stamp = rospy.Time.now()
        self.detections_pub.publish(DetectionArray(h, clean_detections))
        self.image_pub.publish(image_msg)

        # Publish TF
        t = TransformStamped()
        t.header = depth_data.header
        t.child_frame_id = self.bbox_name
        t.transform.translation.x = obj[0]
        t.transform.translation.y = obj[1]
        t.transform.translation.z = obj[2]
        # compute the tf frame
        # rotate -90 degrees along z-axis
        t.transform.rotation.z = np.sin(-np.pi / 4)
        t.transform.rotation.w = np.cos(-np.pi / 4)
        self._br.sendTransform([t])


if __name__ == "__main__":
    rospy.init_node("bbox_publisher")
    img_topic = "/hsrb/head_rgbd_sensor/rgb/image_rect_color"
    depth_topic = "/hsrb/head_rgbd_sensor/depth_registered/image_rect_raw"
    bbox_name = "target"
    sub = BboxPublisher(img_topic, depth_topic, bbox_name=bbox_name)
    rospy.spin()
