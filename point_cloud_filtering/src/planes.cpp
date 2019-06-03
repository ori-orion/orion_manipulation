//
// Created by Mark Finean on 2019-05-16.
//
#include "planes.h"

#include "pcl_conversions/pcl_conversions.h"

#include "pcl/PointIndices.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"

#include "pcl/common/angles.h"
#include "pcl/sample_consensus/method_types.h"
#include "pcl/sample_consensus/model_types.h"
#include "pcl/segmentation/sac_segmentation.h"
#include "pcl/segmentation/extract_clusters.h"

#include "pcl/filters/extract_indices.h"

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include "pcl/filters/crop_box.h"


typedef pcl::PointXYZRGB PointC;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudC;

namespace point_cloud_filtering {

void SegmentSurface(PointCloudC::Ptr cloud, pcl::PointIndices::Ptr indices) {
  
  pcl::PointIndices indices_internal;
  pcl::SACSegmentation<PointC> seg;
  seg.setOptimizeCoefficients(true);

  // Search for a plane perpendicular to some axis (specified below).
  seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);

  // Set the distance to the plane for a point to be an inlier.
  seg.setDistanceThreshold(0.01);
  seg.setInputCloud(cloud);

  // Make sure that the plane is perpendicular to X-axis, 10 degree tolerance.
  Eigen::Vector3f axis;
  axis << 0, 1, 0;
  seg.setAxis(axis);
  seg.setEpsAngle(pcl::deg2rad(5.0));

  // coeff contains the coefficients of the plane:
  // ax + by + cz + d = 0
  pcl::ModelCoefficients coeff;
  seg.segment(indices_internal, coeff);

  *indices = indices_internal;

  if (indices->indices.empty()) {
    ROS_ERROR("Unable to find surface.");
    return;
  }
}

void CropCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr in_cloud,
	       pcl::PointCloud<pcl::PointXYZRGB>::Ptr out_cloud,
               Eigen::Vector4f min_p,
               Eigen::Vector4f max_p){

  pcl::CropBox<PointC> crop;
  crop.setInputCloud(in_cloud);
  crop.setMin(min_p);
  crop.setMax(max_p);
  crop.filter(*out_cloud);
}

void SegmentDoorInliers(PointCloudC::Ptr in_cloud, 
		     pcl::PointIndices::Ptr points_within){

    pcl::SACSegmentation<pcl::PointXYZRGB> seg;
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
    
    // Optional
    seg.setOptimizeCoefficients (true);

    // Mandatory
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (1000);
    seg.setDistanceThreshold (0.01);
    seg.setInputCloud (in_cloud);
    seg.segment (*points_within, *coefficients); 
}

void GetHandleFromClusters(std::vector<pcl::PointIndices>* clusters, 
			   PointCloudC::Ptr in_cloud, 
		           PointCloudC::Ptr out_cloud){

  pcl::PointIndices::Ptr handle_inliers(new pcl::PointIndices());
  pcl::ExtractIndices<PointC> handle_extract;

  int clust_size, ind;
  clust_size = 0; 
  ind = 0;
  for(size_t i=0; i < clusters->size(); ++i) {
    if(clusters->at(i).indices.size() > clust_size){
      ind = i;
    }
  }

  *handle_inliers = clusters->at(ind);
  handle_extract.setInputCloud(in_cloud);
  handle_extract.setIndices(handle_inliers);
  handle_extract.filter(*out_cloud);

}

void GetClusters(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
                 std::vector<pcl::PointIndices>* clusters){
  double cluster_tolerance;
  int min_cluster_size, max_cluster_size;
  ros::param::param("ec_cluster_tolerance", cluster_tolerance, 0.01);
  ros::param::param("ec_min_cluster_size", min_cluster_size, 30);
  ros::param::param("ec_max_cluster_size", max_cluster_size, 100);

  pcl::EuclideanClusterExtraction<PointC> euclid;
  euclid.setInputCloud(cloud);
  euclid.setClusterTolerance(cluster_tolerance);
  euclid.setMinClusterSize(min_cluster_size);
  euclid.setMaxClusterSize(max_cluster_size);
  euclid.extract(*clusters);
}

void RemoveDoor(PointCloudC::Ptr in_cloud, PointCloudC::Ptr out_cloud, pcl::PointIndices::Ptr door_inliers){
        pcl::ExtractIndices<PointC> extract;
        extract.setInputCloud(in_cloud);
        extract.setIndices(door_inliers);
        extract.setNegative(true);
        extract.filter(*out_cloud);
}

HandleCropper::HandleCropper(const ros::Publisher& pub) : pub_(pub) {}

void HandleCropper::Callback(const sensor_msgs::PointCloud2& msg) {

  // Check the incoming point cloud
  PointCloudC::Ptr cloud(new PointCloudC());
  pcl::fromROSMsg(msg, *cloud);
  ROS_INFO("Got point cloud with %ld points", cloud->size());

  //------ Get the door --------
  // Implement dominant plane filter - Create the segmentation object 
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
  SegmentDoorInliers(cloud, inliers);

  if (inliers->indices.empty ())
  {
    PCL_ERROR ("Could not estimate a planar model for the given dataset.");
  }

  // Extract the plane indices subset of cloud into output_cloud:
  pcl::ExtractIndices<PointC> door_extract;
  PointCloudC::Ptr door_cloud (new PointCloudC());
  door_extract.setInputCloud(cloud);
  door_extract.setIndices(inliers);
  door_extract.filter(*door_cloud);


  float min_x = std::numeric_limits<float>::max();
  float min_y = std::numeric_limits<float>::max();
  float min_z = std::numeric_limits<float>::max();
  float max_x = std::numeric_limits<float>::min();
  float max_y = std::numeric_limits<float>::min();
  float max_z = std::numeric_limits<float>::min();

  for(size_t i=0; i < door_cloud->points.size(); ++i) {
    if (door_cloud->points[i].x < min_x) {
      min_x = door_cloud->points[i].x;
    }
    if (door_cloud->points[i].y < min_y) {
      min_y = door_cloud->points[i].y;
    }
    if (door_cloud->points[i].z < min_z) {
      min_z = door_cloud->points[i].z;
    }
    if (door_cloud->points[i].x > max_x) {
      max_x = door_cloud->points[i].x;
    }
    if (door_cloud->points[i].y > max_y) {
      max_y = door_cloud->points[i].y;
    }
    if (door_cloud->points[i].z > max_z) {
      max_z = door_cloud->points[i].z;
    }
  }

  // Remove the door
  PointCloudC::Ptr filtered_cloud(new PointCloudC());
  RemoveDoor(cloud, filtered_cloud, inliers);


  //------ Crop the point cloud used to get the handle --------
  Eigen::Vector4f min_pt(min_x+0.13, min_y+0.1, min_z-0.12, 1);
  Eigen::Vector4f max_pt(max_x-0.13, max_y-0.1, max_z+0.05, 1);
  PointCloudC::Ptr cropped_cloud(new PointCloudC());
  CropCloud(filtered_cloud, cropped_cloud, min_pt, max_pt);

  // Extract the handle indices
  std::vector<pcl::PointIndices> protruding_clusters;
  //GetDoorClusters(cropped_cloud, inliers, &protruding_clusters); 
  GetClusters(cropped_cloud, &protruding_clusters);  
  std::cout << "Door Clusters succeeded." << std::endl;
  
  ROS_INFO("There are %ld clusters", protruding_clusters.size());
  if(not protruding_clusters.empty() > 0){
     PointCloudC::Ptr handle_cloud(new PointCloudC);
     GetHandleFromClusters(&protruding_clusters, cropped_cloud, handle_cloud);

     // Publish output cloud
     sensor_msgs::PointCloud2 msg_out;
     pcl::toROSMsg(*handle_cloud, msg_out);
     pub_.publish(msg_out);
   }
  else{
     std::cout << "No clusters detected. Publishing door" << std::endl;
     sensor_msgs::PointCloud2 msg_out;
     pcl::toROSMsg(*cropped_cloud, msg_out);
     pub_.publish(msg_out);
  }
}

} //namespace point_cloud_filtering
