//
// Original author: Mark Finean
// Maintainer: Matthew Budd
//

#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"

namespace point_cloud_filtering {
class Cropper {
 public:
  Cropper(const ros::Publisher& pub);
  void Callback(const sensor_msgs::PointCloud2& msg);

 private:
  ros::Publisher pub_;
};
}  // namespace point_cloud_filtering
