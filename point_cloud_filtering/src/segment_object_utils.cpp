//
// Original author: Mark Finean
// Maintainer: Matthew Budd
//

#include "segment_object_utils.h"

#include <eigen_conversions/eigen_msg.h>
#include <visualization_msgs/Marker.h>

#include "geometry_msgs/PointStamped.h"
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"

typedef pcl::PointXYZRGB PointC;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudC;

namespace point_cloud_filtering {

ObjectSegmenter::ObjectSegmenter(const ros::Publisher& object_pub, const double x_in,
                                 const double y_in, const double z_in) {
  object_pub_ = object_pub;
  object_x = x_in;
  object_y = y_in;
  object_z = z_in;
  ros::NodeHandle nh;

  marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 10);
  crop_pub = nh.advertise<sensor_msgs::PointCloud2>("cropped_cloud", 1, true);
}

void ObjectSegmenter::Callback(const sensor_msgs::PointCloud2& msg) {
  // Check the incoming point cloud
  PointCloudC::Ptr cloud(new PointCloudC());
  pcl::fromROSMsg(msg, *cloud);
  ROS_INFO("Got point cloud with %ld points", cloud->size());

  visualization_msgs::Marker marker;
  marker.header.frame_id = "head_rgbd_sensor_rgb_frame";
  marker.header.stamp = ros::Time();
  marker.ns = "my_namespace";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::SPHERE;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position.x = object_x;
  marker.pose.position.y = object_y;
  marker.pose.position.z = object_z;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 0.1;
  marker.scale.y = 0.1;
  marker.scale.z = 0.1;
  marker.color.a = 1.0;
  marker.color.r = 0.0;
  marker.color.g = 1.0;
  marker.color.b = 0.0;
  marker_pub.publish(marker);

  //------ Crop the point cloud roughly 20 cm around the object--------
  float crop_radius = 0.17;
  Eigen::Vector4f min_crop_pt(object_x - crop_radius, object_y - crop_radius,
                              object_z - crop_radius, 1);
  Eigen::Vector4f max_crop_pt(object_x + crop_radius, object_y + crop_radius,
                              object_z + crop_radius, 1);

  PointCloudC::Ptr first_cropped_cloud(new PointCloudC());
  CropCloud(cloud, first_cropped_cloud, min_crop_pt, max_crop_pt);

  ROS_INFO("Cropped cloud has cloud with %ld points", first_cropped_cloud->size());

  //------ Get the table in the point cloud --------
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
  SegmentTable(first_cropped_cloud, inliers);

  if (inliers->indices.empty()) {
    PCL_ERROR("Could not estimate a planar model for the given dataset.");
  }

  // Extract the plane indices subset of cloud into output_cloud:
  pcl::ExtractIndices<PointC> table_extract;
  PointCloudC::Ptr table_cloud(new PointCloudC());
  table_extract.setInputCloud(first_cropped_cloud);
  table_extract.setIndices(inliers);
  table_extract.filter(*table_cloud);

  // Remove the door
  PointCloudC::Ptr no_surface_cloud(new PointCloudC());
  RemoveSurface(first_cropped_cloud, no_surface_cloud, inliers);

  sensor_msgs::PointCloud2 msg_cloud_out;
  pcl::toROSMsg(*no_surface_cloud, msg_cloud_out);
  ROS_INFO("Publishing no_surface_cloud");
  crop_pub.publish(msg_cloud_out);

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
  ROS_INFO("min_y= %f ", min_y);
  ROS_INFO("max_y= %f ", max_y);
  //------ Crop the point cloud used to get the handle --------
  Eigen::Vector4f min_pt(object_x - crop_radius, object_y - crop_radius,
                         object_z - crop_radius, 1);
  Eigen::Vector4f max_pt(object_x + crop_radius, min_y + 0.15, object_z + crop_radius, 1);
  PointCloudC::Ptr object_cloud(new PointCloudC());
  CropCloud(no_surface_cloud, object_cloud, min_pt, max_pt);
  ROS_INFO("min_pt=");
  std::cout << min_pt << std::endl;
  ROS_INFO("max_pt=");
  std::cout << max_pt << std::endl;

  // Publish the object point cloud
  pcl::toROSMsg(*object_cloud, msg_cloud_out);
  object_pub_.publish(msg_cloud_out);
}

void SegmentTable(PointCloudC::Ptr cloud, pcl::PointIndices::Ptr indices) {
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
