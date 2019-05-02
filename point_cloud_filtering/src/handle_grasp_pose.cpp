#include "point_cloud_filtering/crop.h"
#include "point_cloud_filtering/planes.h"
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "handle_grasp_pose");
  ros::NodeHandle nh;

  // To publish cropped door and handle
  ros::Publisher handle_pub =
      nh.advertise<sensor_msgs::PointCloud2>("handle_cloud", 1, true);

  // Crop for the handle
  point_cloud_filtering::HandleCropper handle_cropper(handle_pub);
  ros::Subscriber sub_door =
      nh.subscribe("cloud_in", 1, &point_cloud_filtering::HandleCropper::Callback, &handle_cropper);

  ros::spin();
  return 0;
}
