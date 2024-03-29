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

class HandleCropper {
 public:
  HandleCropper(const ros::Publisher& cloud_pub, const ros::Publisher& door_pub);
  void Callback(const sensor_msgs::PointCloud2& msg);

 private:
  ros::Publisher cloud_pub_;
  ros::Publisher door_pub_;
};

class HandleCentroid {
 public:
  HandleCentroid(const tf::TransformBroadcaster& br);
  void Callback(const sensor_msgs::PointCloud2& msg);
  bool CheckDetection();
  double GetX();
  double GetY();
  double GetZ();

 private:
  //        ros::Publisher handle_centroid_pub_;
  tf::TransformBroadcaster handle_tf_br_;
  bool good_detection_;
  double x_, y_, z_;
};

class DrawerHandleCropper {
 public:
  DrawerHandleCropper(const ros::Publisher& cloud_pub, const ros::Publisher& plane_pub);
  void Callback(const sensor_msgs::PointCloud2& msg);
  void GetHeadAngle(const sensor_msgs::JointState& msg);

 private:
  ros::Publisher cloud_pub_;
  ros::Publisher plane_pub_;
  double head_angle;
};

class DrawerHandleCentroid {
 public:
  DrawerHandleCentroid(const tf::TransformBroadcaster& br);
  void Callback(const sensor_msgs::PointCloud2& msg);
  bool CheckDetection();
  std::vector<double> GetX();
  std::vector<double> GetY();
  std::vector<double> GetZ();

 private:
  tf::TransformBroadcaster handle_tf_br_;
  bool good_detection_;
  std::vector<double> x_, y_, z_;
};
}  // namespace point_cloud_filtering
