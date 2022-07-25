//
// Original author: Mark Finean
// Maintainer: Matthew Budd
//

#include "point_cloud_filtering/surface_utils.h"

#include <eigen_conversions/eigen_msg.h>

#include "geometry_msgs/PointStamped.h"

namespace point_cloud_filtering {

ObjectSegmenter::SurfacePlacement(const ros::Publisher& goal_pub) : goal_pub_(goal_pub) {}

void SurfacePlacement::GetHeadAngle(const sensor_msgs::JointState& msg) {
  head_angle = msg.position[13];
}

void SurfacePlacement::Callback(const sensor_msgs::PointCloud2& msg) {
  // Check the incoming point cloud
  PointCloudC::Ptr cloud(new PointCloudC());
  pcl::fromROSMsg(msg, *cloud);
  ROS_INFO("Got point cloud with %ld points", cloud->size());

  //------ Crop the point cloud roughly 20 cm around the object--------
  Eigen::Vector4f min_crop_pt(-0.2, 0.0, 0, 1);
  Eigen::Vector4f max_crop_pt(0.2, 1.0, 2, 1);
  PointCloudC::Ptr first_cropped_cloud(new PointCloudC());
  CropCloud(cloud, first_cropped_cloud, min_crop_pt, max_crop_pt);

  ROS_INFO("First cropping cloud has cloud with %ld points", first_cropped_cloud->size());

  //------ Get the table in the point cloud --------
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
  Eigen::Vector3f axis;
  axis << 0, 1 - sin(head_angle), 0 + cos(head_angle);

  SegmentTable(first_cropped_cloud, inliers, axis);

  if (inliers->indices.empty()) {
    PCL_ERROR("Could not estimate a planar model for the given dataset.");
  }

  // Extract the plane indices subset of cloud into table_cloud:
  pcl::ExtractIndices<PointC> table_extract;
  PointCloudC::Ptr table_cloud(new PointCloudC());
  table_extract.setInputCloud(first_cropped_cloud);
  table_extract.setIndices(inliers);
  table_extract.filter(*table_cloud);

  // Remove the door
  PointCloudC::Ptr no_surface_cloud(new PointCloudC());
  RemoveSurface(first_cropped_cloud, no_surface_cloud, inliers);

  // At this point the object cloud may have anything below the table still in.
  // The y axis points towards the floor so need to crop anything above max y value in the
  // table inliers.

  float max_y = std::numeric_limits<float>::min();
  float min_y = std::numeric_limits<float>::max();

  for (size_t i = 0; i < table_cloud->points.size(); ++i) {
    if (table_cloud->points[i].y > max_y) {
      max_y = table_cloud->points[i].y;
    }
    if (table_cloud->points[i].y < min_y) {
      min_y = table_cloud->points[i].y;
    }
  }

  //------ Crop the point cloud used to get all objects above the surface -------
  Eigen::Vector4f min_crop_pt(-0.2, 0.0, 0, 1);
  Eigen::Vector4f max_crop_pt(0.2, max_y, 2, 1);

  PointCloudC::Ptr final_cloud(new PointCloudC());
  CropCloud(no_surface_cloud, final_cloud, min_pt, max_pt);

  // Publish the object point cloud
  if (not inliers->indices.empty()) {
    sensor_msgs::PointCloud2 msg_cloud_out;
    pcl::toROSMsg(*final_cloud, msg_cloud_out);
    goal_pub_.publish(msg_cloud_out);
  }

  // TO DO: This need to eliminate all points on the surface where there is an x,y
  // coordinate in the object point cloud.
  pcl::PointIndices::Ptr surface_inliers_to_remove(new pcl::PointIndices());
  pcl::ExtractIndices<PointC> point_extracter;

  for (int i = 0; i < (*p_obstacles).size(); i++) {
    pcl::PointXYZ pt(p_obstacles->points[i].x, p_obstacles->points[i].y,
                     p_obstacles->points[i].z);
    float zAvg = 0.5f;
    if (abs(pt.z - zAvg) < THRESHOLD) {
      // e.g. remove all pts below zAvg
      inliers->indices.push_back(i);
    }
  }
  extract.setInputCloud(p_obstacles);
  extract.setIndices(inliers);
  extract.setNegative(true);
  extract.filter(*p_obstacles);
}

void SegmentTable(PointCloudC::Ptr cloud, pcl::PointIndices::Ptr indices,
                  Eigen::Vector3f axis) {
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
  seg.setAxis(axis);
  seg.setEpsAngle(pcl::deg2rad(20.0));

  // coeff contains the coefficients of the plane:
  // ax + by + cz + d = 0
  pcl::ModelCoefficients coeff;
  seg.segment(indices_internal, coeff);

  *indices = indices_internal;

  if (indices->indices.empty()) {
    ROS_ERROR("Unable to find surface.");
  }
}

void RemoveSurface(PointCloudC::Ptr in_cloud, PointCloudC::Ptr out_cloud,
                   pcl::PointIndices::Ptr inliers) {
  pcl::ExtractIndices<PointC> extract;
  extract.setInputCloud(in_cloud);
  extract.setIndices(inliers);
  extract.setNegative(true);
  extract.filter(*out_cloud);
}

void CropCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr in_cloud,
               pcl::PointCloud<pcl::PointXYZRGB>::Ptr out_cloud, Eigen::Vector4f min_p,
               Eigen::Vector4f max_p) {
  pcl::CropBox<PointC> crop;
  crop.setInputCloud(in_cloud);
  crop.setMin(min_p);
  crop.setMax(max_p);
  crop.filter(*out_cloud);
}

}  // namespace point_cloud_filtering
