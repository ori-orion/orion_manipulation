//
// Original author: Mark Finean
// Maintainer: Matthew Budd
//

#include <tf/transform_broadcaster.h>

#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "sensor_msgs/PointCloud2.h"

namespace point_cloud_filtering {

class BinHandleCropper {
 public:
  BinHandleCropper(const ros::Publisher& bin_handle_pub,
                   const ros::Publisher& bin_surface_pub);
  void Callback(const sensor_msgs::PointCloud2& msg);

 private:
  ros::Publisher bin_handle_pub_;
  ros::Publisher bin_surface_pub_;
};

class BinHandleCentroid {
 public:
  BinHandleCentroid(const tf::TransformBroadcaster& br);
  void Callback(const sensor_msgs::PointCloud2& msg);
  bool CheckDetection();
  double GetX();
  double GetY();
  double GetZ();

 private:
  tf::TransformBroadcaster bin_handle_tf_br_;
  bool good_detection_;
  double x_, y_, z_;
};

}  // namespace point_cloud_filtering
