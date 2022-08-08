//
// Original author: Mark Finean
// Maintainer: Kim Tien Ly
//

#include "segment_surface_utils.h"

#include <eigen_conversions/eigen_msg.h>
#include <visualization_msgs/Marker.h>

#include "geometry_msgs/Point.h"
#include "geometry_msgs/PointStamped.h"
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"

typedef pcl::PointXYZRGB PointC;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudC;

namespace point_cloud_filtering {

SurfaceSegmenter::SurfaceSegmenter(const ros::Publisher& object_pub, const double x_in,
                                   const double y_in, const double z_in) {
  object_pub_ = object_pub;
  query_point << x_in, y_in, z_in;

  ros::NodeHandle nh;

  placeholder_pub = nh.advertise<geometry_msgs::Point>("placeholder", 10);

  visual_tools.reset(new rviz_visual_tools::RvizVisualTools("head_rgbd_sensor_rgb_frame",
                                                            "vis_markers"));
  visual_tools->trigger();
}

void SurfaceSegmenter::Callback(const sensor_msgs::PointCloud2& msg) {
  // Check the incoming point cloud
  PointCloudC::Ptr cloud(new PointCloudC());
  pcl::fromROSMsg(msg, *cloud);
  ROS_INFO("Got point cloud with %ld points", cloud->size());

  //------ Crop the point cloud roughly 15 cm around the object--------
  float crop_radius = 0.15;
  Eigen::Vector4f min_crop_pt(query_point[0] - crop_radius, query_point[1] - crop_radius,
                              query_point[2] - crop_radius, 1);
  Eigen::Vector4f max_crop_pt(query_point[0] + crop_radius, query_point[1] + crop_radius,
                              query_point[2] + crop_radius, 1);

  PublishCropBoundingBoxMarker(min_crop_pt, max_crop_pt);

  PointCloudC::Ptr first_cropped_cloud(new PointCloudC());
  CropCloud(cloud, first_cropped_cloud, min_crop_pt, max_crop_pt);

  ROS_INFO("segment_utils: Cropped cloud has cloud with %ld points",
           first_cropped_cloud->size());

  //------ Get the surface in the point cloud --------
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
  pcl::ModelCoefficients::Ptr plane_coeff(new pcl::ModelCoefficients);
  SegmentPlane(first_cropped_cloud, inliers, plane_coeff);

  if (inliers->indices.empty()) {
    PCL_ERROR("segment_utils: Could not estimate a planar model for the given dataset.");
    return;
  }

  Eigen::Vector3d plane_projection;
  CalculatePlaneProjection(plane_coeff, query_point, plane_projection);
  ROS_INFO_STREAM("segment_utils: Calculated plane point:  " << plane_projection);
  PublishPlaneMarker(plane_coeff, plane_projection);

  // Publish the plane projection point
  geometry_msgs::Point placeholder_msg;
  tf::pointEigenToMsg(plane_projection, placeholder_msg);
  placeholder_pub.publish(placeholder_msg);

  // Extract the plane indices subset of cloud into output_cloud:
  pcl::ExtractIndices<PointC> table_extract;
  PointCloudC::Ptr table_cloud(new PointCloudC());
  table_extract.setInputCloud(first_cropped_cloud);
  table_extract.setIndices(inliers);
  table_extract.filter(*table_cloud);

  PointCloudC::Ptr surface_cloud(new PointCloudC());
  GetSurface(first_cropped_cloud, surface_cloud, inliers);

  sensor_msgs::PointCloud2 msg_cloud_out;
  pcl::toROSMsg(*surface_cloud, msg_cloud_out);
  object_pub_.publish(msg_cloud_out);
}

void SurfaceSegmenter::CalculatePlaneProjection(pcl::ModelCoefficients::Ptr plane_coeff,
                                                Eigen::Vector3d point,
                                                Eigen::Vector3d& closest_point) {
  // Calculate the closest point to a point on a plane.
  // ax + by + cz + d = 0
  float a = plane_coeff->values[0];
  float b = plane_coeff->values[1];
  float c = plane_coeff->values[2];
  float d = plane_coeff->values[3];
  Eigen::Vector3d plane_abc = Eigen::Vector3d(a, b, c);

  float k = (-a * point[0] - b * point[1] - c * point[2] - d) /
            static_cast<float>(a * a + b * b + c * c);

  closest_point = point + (plane_abc * k);
}

void SurfaceSegmenter::PublishCropBoundingBoxMarker(Eigen::Vector4f min_crop_pt,
                                                    Eigen::Vector4f max_crop_pt) {
  Eigen::Isometry3d identity_pose = Eigen::Isometry3d::Identity();

  ROS_INFO("Publishing bounding box marker");
  visual_tools->deleteAllMarkers();
  visual_tools->publishWireframeCuboid(
      identity_pose, min_crop_pt.head<3>().cast<double>(),
      max_crop_pt.head<3>().cast<double>(), rviz_visual_tools::BLUE);
  visual_tools->trigger();
}

void SurfaceSegmenter::PublishPlaneMarker(pcl::ModelCoefficients::Ptr plane_coeff,
                                          Eigen::Vector3d plane_projection) {
  // Publish normal vector marker

  Eigen::Isometry3d plane_pose;

  plane_pose.setIdentity();
  Eigen::Vector3d plane_normal = Eigen::Vector3d(
      plane_coeff->values[0], plane_coeff->values[1], plane_coeff->values[2]);
  Eigen::Quaterniond rotQ =
      Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d::UnitX(), plane_normal);
  plane_pose.rotate(rotQ);

  plane_pose.translation() = plane_projection;

  visual_tools->publishYZPlane(plane_pose, rviz_visual_tools::BLUE, 0.15);
  visual_tools->publishArrow(plane_pose, rviz_visual_tools::BLUE,
                             rviz_visual_tools::SMALL);
  visual_tools->trigger();
}

void SegmentPlane(PointCloudC::Ptr cloud, pcl::PointIndices::Ptr indices,
                  pcl::ModelCoefficients::Ptr coeff) {
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
  pcl::ModelCoefficients plane_coeff;
  seg.segment(indices_internal, plane_coeff);

  *coeff = plane_coeff;
  *indices = indices_internal;

  if (indices->indices.empty()) {
    ROS_ERROR("Unable to find surface.");
  }
}

void GetSurface(PointCloudC::Ptr in_cloud, PointCloudC::Ptr out_cloud,
                pcl::PointIndices::Ptr inliers) {
  pcl::ExtractIndices<PointC> extract;
  extract.setInputCloud(in_cloud);
  extract.setIndices(inliers);
  extract.setNegative(false);
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
