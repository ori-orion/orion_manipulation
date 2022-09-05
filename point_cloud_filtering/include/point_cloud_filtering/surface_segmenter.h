//
// Original author: Mark Finean
// Maintainer: Kim Tien Ly
//

#ifndef POINT_CLOUD_FILTERING_INCLUDE_POINT_CLOUD_FILTERING_SURFACE_SEGMENTER_H_
#define POINT_CLOUD_FILTERING_INCLUDE_POINT_CLOUD_FILTERING_SURFACE_SEGMENTER_H_

#include <pcl/ModelCoefficients.h>
#include <pcl/filters/plane_clipper3D.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rviz_visual_tools/rviz_visual_tools.h>
#include <tf/transform_broadcaster.h>

#include <vector>

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
#include "point_cloud_filtering/DetectSurface.h"

typedef pcl::PointXYZRGB PointC;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudC;

namespace point_cloud_filtering {

void CropCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr in_cloud,
               pcl::PointCloud<pcl::PointXYZRGB>::Ptr out_cloud, Eigen::Vector4f min_p,
               Eigen::Vector4f max_p);

void FilterByPlane(pcl::PointCloud<pcl::PointXYZRGB>::Ptr in_cloud,
                   pcl::ModelCoefficients::Ptr plane_coeff,
                   pcl::PointIndices::Ptr plane_side_points);

Eigen::Isometry3d GetPlanePose(pcl::ModelCoefficients::Ptr plane_coeff,
                               Eigen::Vector3d plane_projection);

void SegmentPlane(PointCloudC::Ptr cloud, pcl::PointIndices::Ptr indices,
                  Eigen::Vector3f axis, float eps_degrees_tolerance,
                  pcl::ModelCoefficients::Ptr coeff);

void FilterCloudByIndices(PointCloudC::Ptr in_cloud, PointCloudC::Ptr out_cloud,
                          pcl::PointIndices::Ptr indices, bool invert = false);

Eigen::Vector4f ToHomogeneousCoordsVector(const Eigen::Vector3d in);

void GetCloudMinMaxX(const PointCloudC::Ptr cloud, float& min_x, float& max_x);
void GetCloudMinMaxY(const PointCloudC::Ptr cloud, float& min_y, float& max_y);
void GetCloudMinMaxZ(const PointCloudC::Ptr cloud, float& min_z, float& max_z);

void ClusterCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
                  std::vector<pcl::PointIndices>* clusters,
                  double cluster_tolerance = 0.01, double min_cluster_proportion = 0.05,
                  double max_cluster_proportion = 1.0);

void GetLargestCluster(std::vector<pcl::PointIndices>* clusters,
                       PointCloudC::Ptr in_cloud, PointCloudC::Ptr out_cloud);

class SurfaceSegmenter {
 public:
  explicit SurfaceSegmenter(ros::NodeHandle* nh);

 private:
  ros::NodeHandle* ros_node_handle;
  ros::ServiceServer service_server;
  ros::Publisher object_pub;
  ros::Publisher placeholder_pub;

  rviz_visual_tools::RvizVisualToolsPtr visual_tools;

  bool ServiceCallback(point_cloud_filtering::DetectSurface::Request& req,
                       point_cloud_filtering::DetectSurface::Response& res);

  void CalculatePlaneProjection(pcl::ModelCoefficients::Ptr plane_coeff,
                                Eigen::Vector3d point, Eigen::Vector3d& closest_point);

  void PublishTransformMarker(geometry_msgs::Transform transform);

  void PublishCropBoundingBoxMarker(Eigen::Vector3d min_crop_pt,
                                    Eigen::Vector3d max_crop_pt);

  void PublishPlaneMarker(Eigen::Isometry3d plane_pose, float plane_size = 0.15,
                          float arrow_size = 0.1);
};

}  // namespace point_cloud_filtering

#endif  // POINT_CLOUD_FILTERING_INCLUDE_POINT_CLOUD_FILTERING_SURFACE_SEGMENTER_H_
