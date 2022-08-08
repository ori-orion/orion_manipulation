//
// Original author: Mark Finean
// Maintainer: Kim Tien Ly
//

#include <ros/callback_queue.h>

#include <cstdio>
#include <ctime>
#include <iostream>

#include "geometry_msgs/PointStamped.h"
#include "point_cloud_filtering/SegmentSurface.h"
#include "segment_surface_utils.h"

bool segment_surface(point_cloud_filtering::SegmentSurface::Request &req,
                     point_cloud_filtering::SegmentSurface::Response &res) {
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
  ros::init(argc, argv, "surface_segmentation");
  ros::NodeHandle nh;

  ros::ServiceServer service =
      nh.advertiseService("surface_segmentation", segment_surface);

  ROS_INFO("Ready to segment surface.");

  ros::spin();

  return 0;
}
