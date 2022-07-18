//
// Original author: Mark Finean
// Maintainer: Matthew Budd
//

#include <ros/callback_queue.h>

#include "geometry_msgs/PointStamped.h"
#include "point_cloud_filtering/surface_utils.h"

bool get_surface_location(point_cloud_filtering::GetSurfaceGoal::Request &req,
                          point_cloud_filtering::GetSurfaceGoal::Response &res) {
  ros::NodeHandle nh;
  tf::TransformBroadcaster br;

  // To publish cropped door and handle
  ros::Publisher goal_pub =
      nh.advertise<sensor_msgs::PointCloud2>("valid_surface_cloud", 1, true);

  // Crop for the handle
  point_cloud_filtering::SurfacePlacement surface_placer(goal_pub);

  ros::Subscriber joint_sub = nh.subscribe(
      "/hsrb/robot_state/joint_states", 1,
      &point_cloud_filtering::SurfacePlacement::GetHeadAngle, &surface_placer);

  ros::Subscriber sub_handle = nh.subscribe(
      "cloud_in", 1, &point_cloud_filtering::SurfacePlacement::Callback, &surface_placer);

  return true;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "surface_placement");
  ros::NodeHandle nh;

  ros::ServiceServer service =
      nh.advertiseService("surface_placer", get_surface_location);

  ROS_INFO("Ready to detect handles.");

  ros::spin();

  return 0;
}
