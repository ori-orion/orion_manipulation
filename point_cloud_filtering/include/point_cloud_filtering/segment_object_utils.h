//
// Original author: Mark Finean
// Maintainer: Matthew Budd
//

#include <pcl/ModelCoefficients.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf/transform_broadcaster.h>

#include "pcl/PointIndices.h"
#include "pcl/common/angles.h"
#include "pcl/filters/crop_box.h"
#include "pcl/filters/extract_indices.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl/sample_consensus/method_types.h"
#include "pcl/sample_consensus/model_types.h"
#include "pcl/segmentation/extract_clusters.h"
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"

typedef pcl::PointXYZRGB PointC;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudC;

namespace point_cloud_filtering {

void CropCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr in_cloud,
               pcl::PointCloud<pcl::PointXYZRGB>::Ptr out_cloud, Eigen::Vector4f min_p,
               Eigen::Vector4f max_p);

void SegmentTable(PointCloudC::Ptr cloud, pcl::PointIndices::Ptr indices);

void RemoveSurface(PointCloudC::Ptr in_cloud, PointCloudC::Ptr out_cloud,
                   pcl::PointIndices::Ptr inliers);

class ObjectSegmenter {
 public:
  ObjectSegmenter(const ros::Publisher& object_pub, const double x_in, const double y_in,
                  const double z_in);
  void Callback(const sensor_msgs::PointCloud2& msg);

 private:
  ros::Publisher object_pub_;
  ros::Publisher crop_pub;
  ros::Publisher marker_pub;
  double object_x, object_y, object_z;
};

}  // namespace point_cloud_filtering
