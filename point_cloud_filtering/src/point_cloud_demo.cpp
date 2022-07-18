//
// Original author: Mark Finean
// Maintainer: Matthew Budd
//

#include "point_cloud_filtering/crop.h"
#include "point_cloud_filtering/planes.h"
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "point_cloud_demo");
  ros::NodeHandle nh;

  // To publish a boundary set crop
  ros::Publisher crop_pub =
      nh.advertise<sensor_msgs::PointCloud2>("cropped_cloud", 1, true);

  // To publish cropped door and handle
  ros::Publisher door_pub = nh.advertise<sensor_msgs::PointCloud2>("door_cloud", 1, true);

  // To publish cropped door and handle
  ros::Publisher handle_pub =
      nh.advertise<sensor_msgs::PointCloud2>("handle_pub", 1, true);

  // Crop to set boundaries
  point_cloud_filtering::Cropper cropper(crop_pub);
  ros::Subscriber sub_crop =
      nh.subscribe("cloud_in", 1, &point_cloud_filtering::Cropper::Callback, &cropper);

  // Crop for the door
  point_cloud_filtering::PlaneCropper door_cropper(door_pub);
  ros::Subscriber sub_door = nh.subscribe(
      "cloud_in", 1, &point_cloud_filtering::PlaneCropper::Callback, &door_cropper);

  ros::spin();
  return 0;
}
