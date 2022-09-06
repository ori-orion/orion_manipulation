//
// Original author: Mark Finean
// Maintainer: Kim Tien Ly
//

#ifndef POINT_CLOUD_FILTERING_INCLUDE_POINT_CLOUD_FILTERING_OBJECT_SEGMENTER_H_
#define POINT_CLOUD_FILTERING_INCLUDE_POINT_CLOUD_FILTERING_OBJECT_SEGMENTER_H_

#include "surface_segmenter.h"
#include "point_cloud_filtering/SegmentObject.h"

typedef pcl::PointXYZRGB PointC;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudC;

namespace point_cloud_filtering {

class ObjectSegmenter {
 public:
  ObjectSegmenter(const ros::Publisher& object_pub, const double x_in, const double y_in,
                  const double z_in);
  void Callback(const sensor_msgs::PointCloud2& msg);

 private:
  ros::Publisher object_pub_;
  ros::Publisher crop_pub;
  Eigen::Vector3d query_point;
  rviz_visual_tools::RvizVisualToolsPtr visual_tools;

  void CalculatePlaneProjection(pcl::ModelCoefficients::Ptr plane_coeff,
                                Eigen::Vector3d point,
                                Eigen::Vector3d& closest_point);

  void PublishCropBoundingBoxMarker(Eigen::Vector3d min_crop_pt,
                                    Eigen::Vector3d max_crop_pt);

  void PublishPlaneMarker(Eigen::Isometry3d plane_pose,
                          float plane_size = 0.15,
                          float arrow_size = 0.1);
};

}  // namespace point_cloud_filtering

#endif  // POINT_CLOUD_FILTERING_INCLUDE_POINT_CLOUD_FILTERING_OBJECT_SEGMENTER_H_
