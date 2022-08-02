//
// Original author: Mark Finean
// Maintainer: Matthew Budd
//

#include <ros/callback_queue.h>

#include "geometry_msgs/PointStamped.h"
#include "sensor_msgs/JointState.h"
#include "handle_segmentation.h"
#include "handle_utils.h"
#include "point_cloud_filtering/DetectDrawerHandles.h"


bool detect_handle(point_cloud_filtering::DetectDrawerHandles::Request &req,
                   point_cloud_filtering::DetectDrawerHandles::Response &res) {
  tf::TransformBroadcaster br;
  ros::NodeHandle nh;

  // To publish cropped door and handle
  ros::Publisher handle_pub =
      nh.advertise<sensor_msgs::PointCloud2>("handle_cloud", 1, true);

  ros::Publisher plane_pub =
      nh.advertise<sensor_msgs::PointCloud2>("plane_cloud", 1, true);

  // Crop for the handle
  point_cloud_filtering::DrawerHandleCropper handle_cropper(handle_pub, plane_pub);
  point_cloud_filtering::DrawerHandleCentroid handle_centroid(br);

  ros::Subscriber joint_sub = nh.subscribe(
      "/hsrb/robot_state/joint_states", 1,
      &point_cloud_filtering::DrawerHandleCropper::GetHeadAngle, &handle_cropper);

  ros::Subscriber sub_handle =
      nh.subscribe("cloud_in", 1, &point_cloud_filtering::DrawerHandleCropper::Callback,
                   &handle_cropper);

  ros::Subscriber sub_handle_centroid = nh.subscribe(
      "handle_cloud", 1, &point_cloud_filtering::DrawerHandleCentroid::Callback,
      &handle_centroid);

  // Until we get a good validated detection of the handle keep looping
  // Need to insert a timer here using argument from high level
  while (not handle_centroid.CheckDetection()) {
    ros::getGlobalCallbackQueue()->callAvailable(ros::WallDuration(0.1));
  }

  res.x = handle_centroid.GetX();
  res.y = handle_centroid.GetY();
  res.z = handle_centroid.GetZ();

  return true;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "drawer_grasp_pose");
  ros::NodeHandle nh;

  ros::ServiceServer service =
      nh.advertiseService("drawer_handle_detection", detect_handle);

  ROS_INFO("Ready to detect drawer handles.");

  ros::spin();

  return 0;
}
