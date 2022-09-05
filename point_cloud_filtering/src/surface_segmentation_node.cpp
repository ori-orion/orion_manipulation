//
// Original author: Mark Finean
// Maintainer: Kim Tien Ly
//

#include <ros/callback_queue.h>
#include <ros/ros.h>

#include <cstdio>
#include <ctime>
#include <iostream>

#include "geometry_msgs/PointStamped.h"
#include "point_cloud_filtering/SegmentSurface.h"
#include "surface_segmenter.h"


int main(int argc, char** argv) {
  ros::init(argc, argv, "surface_segmentation_node");
  ros::NodeHandle nh;

  point_cloud_filtering::SurfaceSegmenter surface_segmenter(&nh);

  ros::spin();

  return 0;
}
