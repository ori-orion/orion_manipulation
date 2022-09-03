//
// Original author: Mark Finean
// Maintainer: Kim Tien Ly
//

#include <ros/callback_queue.h>

#include <cstdio>
#include <ctime>
#include <iostream>

#include "geometry_msgs/PointStamped.h"
#include "point_cloud_filtering/SegmentObject.h"
#include "object_segmenter.h"

bool segment_object(point_cloud_filtering::SegmentObject::Request &req,
                    point_cloud_filtering::SegmentObject::Response &res) {
  ros::NodeHandle nh;
  tf::TransformBroadcaster br;

  // To publish the segmented object
  ros::Publisher object_pub =
      nh.advertise<sensor_msgs::PointCloud2>("object_cloud", 1, true);

  // Instantiate the object segmenter
  point_cloud_filtering::ObjectSegmenter object_segmenter(object_pub, req.x, req.y,
                                                          req.z);

  // Segment the object when you get a point cloud
  ros::Subscriber sub_handle =
      nh.subscribe("cloud_in", 1, &point_cloud_filtering::ObjectSegmenter::Callback,
                   &object_segmenter);

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
  ros::init(argc, argv, "object_segmentation");
  ros::NodeHandle nh;

  ros::ServiceServer service = nh.advertiseService("object_segmentation", segment_object);

  ROS_INFO("Ready to segment objects.");

  ros::spin();

  return 0;
}
