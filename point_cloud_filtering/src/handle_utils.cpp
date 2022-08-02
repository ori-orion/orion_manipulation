//
// Original author: Mark Finean
// Maintainer: Matthew Budd
//

#include "handle_utils.h"

namespace point_cloud_filtering {

void CropCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr in_cloud,
               pcl::PointCloud<pcl::PointXYZRGB>::Ptr out_cloud, Eigen::Vector4f min_p,
               Eigen::Vector4f max_p) {
  pcl::CropBox<PointC> crop;
  crop.setInputCloud(in_cloud);
  crop.setMin(min_p);
  crop.setMax(max_p);
  crop.filter(*out_cloud);
}

void SegmentDoorInliers(PointCloudC::Ptr in_cloud, pcl::PointIndices::Ptr points_within) {
  pcl::SACSegmentation<pcl::PointXYZRGB> seg;
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());

  // Optional
  seg.setOptimizeCoefficients(true);

  // Mandatory
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setMaxIterations(1000);
  seg.setDistanceThreshold(0.01);
  seg.setInputCloud(in_cloud);
  seg.segment(*points_within, *coefficients);
}

void SegmentBinSurfaceInliers(PointCloudC::Ptr cloud, pcl::PointIndices::Ptr indices) {
  pcl::PointIndices indices_internal;
  pcl::SACSegmentation<PointC> seg;
  seg.setOptimizeCoefficients(true);

  // Search for a plane perpendicular to some axis (specified below).
  seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);

  // Set the distance to the plane for a point to be an inlier.
  seg.setDistanceThreshold(0.03);
  seg.setInputCloud(cloud);

  // Make sure that the plane is perpendicular to X-axis, 35 degree tolerance.
  Eigen::Vector3f axis;
  axis << 0, 1, 0;
  seg.setAxis(axis);
  seg.setEpsAngle(pcl::deg2rad(35.0));

  // coeff contains the coefficients of the plane:
  // ax + by + cz + d = 0
  pcl::ModelCoefficients coeff;
  seg.segment(indices_internal, coeff);

  *indices = indices_internal;

  if (indices->indices.empty()) {
    ROS_ERROR("Unable to find surface.");
  }
}

void SegmentFurnitureInliers(PointCloudC::Ptr in_cloud,
                             pcl::PointIndices::Ptr points_within,
                             pcl::ModelCoefficients::Ptr coefficients,
                             Eigen::Vector3f axis) {
  pcl::SACSegmentation<pcl::PointXYZRGB> seg;

  // Optional
  seg.setOptimizeCoefficients(true);

  // Mandatory
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setMaxIterations(1000);
  seg.setDistanceThreshold(0.01);

  seg.setAxis(axis);
  seg.setEpsAngle(pcl::deg2rad(5.0));

  seg.setInputCloud(in_cloud);
  seg.segment(*points_within, *coefficients);

  std::cerr << "Model coefficients: " << coefficients->values[0] << " "
            << coefficients->values[1] << " " << coefficients->values[2] << " "
            << coefficients->values[3] << std::endl;
}

//    void GetHandleFromClusters(std::vector<pcl::PointIndices> *clusters,
//                               PointCloudC::Ptr in_cloud,
//                               PointCloudC::Ptr out_cloud) {
//        pcl::PointIndices::Ptr handle_inliers(new pcl::PointIndices());
//        pcl::ExtractIndices<PointC> handle_extract;
//
//        int clust_size, ind;
//        clust_size = 0;
//        ind = 0;
//        for (size_t i = 0; i < clusters->size(); ++i) {
//            if (clusters->at(i).indices.size() > clust_size) {
//                ind = i;
//            }
//        }
//
//        *handle_inliers = clusters->at(ind);
//        handle_extract.setInputCloud(in_cloud);
//        handle_extract.setIndices(handle_inliers);
//        handle_extract.filter(*out_cloud);
//
//    }
//
void GetClusters(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
                 std::vector<pcl::PointIndices>* clusters) {
  double cluster_tolerance;
  int min_cluster_size, max_cluster_size;
  ros::param::param("ec_cluster_tolerance", cluster_tolerance, 0.06);
  ros::param::param("ec_min_cluster_size", min_cluster_size, 25);
  ros::param::param("ec_max_cluster_size", max_cluster_size, 300);

  std::cout << "Getting clusters... " << std::endl;

  pcl::EuclideanClusterExtraction<PointC> euclid;
  euclid.setInputCloud(cloud);
  euclid.setClusterTolerance(cluster_tolerance);
  euclid.setMinClusterSize(min_cluster_size);
  euclid.setMaxClusterSize(max_cluster_size);
  euclid.extract(*clusters);
  std::cout << "Finished clustering... " << std::endl;
}

void RemoveDoor(PointCloudC::Ptr in_cloud, PointCloudC::Ptr out_cloud,
                pcl::PointIndices::Ptr door_inliers) {
  pcl::ExtractIndices<PointC> extract;
  extract.setInputCloud(in_cloud);
  extract.setIndices(door_inliers);
  extract.setNegative(true);
  extract.filter(*out_cloud);
}

}  // namespace point_cloud_filtering
