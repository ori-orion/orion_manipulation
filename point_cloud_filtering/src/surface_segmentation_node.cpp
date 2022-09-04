//
// Original author: Mark Finean
// Maintainer: Kim Tien Ly
//

#include <ros/callback_queue.h>

#include <cstdio>
#include <ctime>
#include <iostream>

#include "geometry_msgs/PointStamped.h"
#include "point_cloud_filtering/DetectSurface.h"
#include "surface_segmenter.h"


bool segment_surface(point_cloud_filtering::DetectSurface::Request &req,
                     point_cloud_filtering::DetectSurface::Response &res) {
  ros::NodeHandle nh;
  tf::TransformBroadcaster br;

  // To publish the segmented surface
  ros::Publisher surface_pub =
      nh.advertise<sensor_msgs::PointCloud2>("surface_cloud", 1, true);

  // Instantiate the object segmenter

  point_cloud_filtering::SurfaceSegmenter surface_segmenter(surface_pub, req.x, req.y,
                                                            req.z);

  // Segment the object when you get a point cloud
  ros::Subscriber sub_handle =
      nh.subscribe("cloud_in", 1, &point_cloud_filtering::SurfaceSegmenter::Callback,
                   &surface_segmenter);

  std::clock_t start;
  double duration = 0;
  start = std::clock();

  while (duration < 5) {
    ros::getGlobalCallbackQueue()->callAvailable(ros::WallDuration(0.1));
    duration = (std::clock() - start) / static_cast<double>CLOCKS_PER_SEC;
  }

  return true;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "surface_segmentation_node");
  ros::NodeHandle nh;

  ros::ServiceServer service =
      nh.advertiseService("detect_surface", segment_surface);

  ROS_INFO("%s: service ready", ros::this_node::getName().c_str());

  ros::spin();

  return 0;
}
