//
// Original author: Mark Finean
// Maintainer: Kim Tien Ly
//

#include "surface_segmenter.h"

#include <eigen_conversions/eigen_msg.h>
#include <visualization_msgs/Marker.h>

#include <cmath>

#include "geometry_msgs/Point.h"
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"

typedef pcl::PointXYZRGB PointC;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudC;

namespace point_cloud_filtering {

SurfaceSegmenter::SurfaceSegmenter(ros::NodeHandle* nh) {
  ros_node_handle = nh;

  object_pub = nh->advertise<sensor_msgs::PointCloud2>("surface_cloud", 1, true);

  service_server =
      nh->advertiseService("detect_surface", &SurfaceSegmenter::ServiceCallback, this);

  visual_tools.reset(new rviz_visual_tools::RvizVisualTools("head_rgbd_sensor_rgb_frame",
                                                            "vis_markers"));
  visual_tools->trigger();
}

bool SurfaceSegmenter::ServiceCallback(
    point_cloud_filtering::DetectSurface::Request& req,
    point_cloud_filtering::DetectSurface::Response& res) {
  res.success = false;

  Eigen::Vector3d query_point;
  query_point << req.x, req.y, req.z;

  //------ Receive one point cloud from RGBD sensor --------
  sensor_msgs::PointCloud2ConstPtr msg =
      ros::topic::waitForMessage<sensor_msgs::PointCloud2>("cloud_in", ros::Duration(3));
  if (msg == NULL) {
    ROS_INFO("No point clound messages received");
    return false;
  }

  PointCloudC::Ptr cloud(new PointCloudC());
  pcl::fromROSMsg(*msg, *cloud);
  ROS_INFO("Received point cloud with %ld points", cloud->size());

  //------ Reset any visualisation --------
  visual_tools->deleteAllMarkers();

  //------ Crop the point cloud roughly 15 cm around the target --------
  float crop_radius = 0.15;
  Eigen::Vector3d min_crop_pt = query_point.array() - crop_radius;
  Eigen::Vector3d max_crop_pt = query_point.array() + crop_radius;
  ROS_INFO_STREAM("Crop points:  " << min_crop_pt << ", " << max_crop_pt);
  PublishCropBoundingBoxMarker(min_crop_pt, max_crop_pt);

  PointCloudC::Ptr first_cropped_cloud(new PointCloudC());
  CropCloud(cloud, first_cropped_cloud, ToHomogeneousCoordsVector(min_crop_pt),
            ToHomogeneousCoordsVector(max_crop_pt));

  ROS_INFO("Box cropped cloud has %ld points", first_cropped_cloud->size());

  //------ Get the surface in the point cloud --------
  pcl::PointIndices::Ptr plane_inliers(new pcl::PointIndices());
  pcl::ModelCoefficients::Ptr plane_coeff(new pcl::ModelCoefficients);
  Eigen::Vector3f axis = Eigen::Vector3f::UnitY();
  float eps_degrees_tolerance = 20.0;

  SegmentPlane(first_cropped_cloud, plane_inliers, axis, eps_degrees_tolerance,
               plane_coeff);

  if (plane_inliers->indices.empty()) {
    ROS_ERROR("Could not estimate a planar model for the given dataset.");
    return false;
  }

  //------ Calculate and publish the plane parameters and visualisation --------
  Eigen::Vector3d plane_projection;
  CalculatePlaneProjection(plane_coeff, query_point, plane_projection);
  ROS_INFO_STREAM("Calculated plane point:  " << plane_projection);
  ROS_INFO_STREAM("Plane parameters:  " << *plane_coeff);
  Eigen::Isometry3d plane_pose = GetPlanePose(plane_coeff, plane_projection);
  PublishPlaneMarker(plane_pose);

  //------ Extract the plane subset of cropped cloud into surface_cloud --------
  PointCloudC::Ptr surface_cloud(new PointCloudC());
  FilterCloudByIndices(first_cropped_cloud, surface_cloud, plane_inliers, false);
  ROS_INFO("Surface cloud has %ld points", surface_cloud->size());

  sensor_msgs::PointCloud2 msg_cloud_out;
  pcl::toROSMsg(*surface_cloud, msg_cloud_out);
  object_pub.publish(msg_cloud_out);

  //------ Publish the plane projection point (should be a service return really) --------
  geometry_msgs::Point placeholder_msg;
  tf::pointEigenToMsg(plane_projection, placeholder_msg);
  res.placeholder = placeholder_msg;
  res.success = true;
  return true;
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

void SurfaceSegmenter::PublishCropBoundingBoxMarker(Eigen::Vector3d min_crop_pt,
                                                    Eigen::Vector3d max_crop_pt) {
  Eigen::Isometry3d identity_pose = Eigen::Isometry3d::Identity();
  visual_tools->publishWireframeCuboid(identity_pose, min_crop_pt, max_crop_pt,
                                       rviz_visual_tools::BLUE);
  visual_tools->trigger();
}

void SurfaceSegmenter::PublishPlaneMarker(Eigen::Isometry3d plane_pose, float plane_size,
                                          float arrow_size) {
  // Publish plane and normal vector markers
  visual_tools->publishXYPlane(plane_pose, rviz_visual_tools::BLUE, plane_size);
  visual_tools->publishZArrow(plane_pose, rviz_visual_tools::BLUE,
                              rviz_visual_tools::SMALL, arrow_size);
  visual_tools->trigger();
}

Eigen::Isometry3d GetPlanePose(pcl::ModelCoefficients::Ptr plane_coeff,
                               Eigen::Vector3d plane_projection) {
  // Generate a pose representing the plane
  Eigen::Vector3d plane_normal = Eigen::Vector3d(
      plane_coeff->values[0], plane_coeff->values[1], plane_coeff->values[2]);
  Eigen::Quaterniond rotQ =
      Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d::UnitZ(), plane_normal);

  Eigen::Isometry3d plane_pose;
  plane_pose.setIdentity();
  plane_pose.rotate(rotQ);
  plane_pose.translation() = plane_projection;

  return plane_pose;
}

void GetCloudMinMaxX(const PointCloudC::Ptr cloud, float& min_x, float& max_x) {
  // Get the min and max x/y/z values of a point cloud
  max_x = std::numeric_limits<float>::min();
  min_x = std::numeric_limits<float>::max();

  for (size_t i = 0; i < cloud->points.size(); ++i) {
    if (cloud->points[i].x > max_x) {
      max_x = cloud->points[i].x;
    }
    if (cloud->points[i].x < min_x) {
      min_x = cloud->points[i].x;
    }
  }
}

void GetCloudMinMaxY(const PointCloudC::Ptr cloud, float& min_y, float& max_y) {
  // Get the min and max x/y/z values of a point cloud
  max_y = std::numeric_limits<float>::min();
  min_y = std::numeric_limits<float>::max();

  for (size_t i = 0; i < cloud->points.size(); ++i) {
    if (cloud->points[i].y > max_y) {
      max_y = cloud->points[i].y;
    }
    if (cloud->points[i].y < min_y) {
      min_y = cloud->points[i].y;
    }
  }
}

void GetCloudMinMaxz(const PointCloudC::Ptr cloud, float& min_z, float& max_z) {
  // Get the min and max x/z/z values of a point cloud
  max_z = std::numeric_limits<float>::min();
  min_z = std::numeric_limits<float>::max();

  for (size_t i = 0; i < cloud->points.size(); ++i) {
    if (cloud->points[i].z > max_z) {
      max_z = cloud->points[i].z;
    }
    if (cloud->points[i].z < min_z) {
      min_z = cloud->points[i].z;
    }
  }
}

void SegmentPlane(PointCloudC::Ptr cloud, pcl::PointIndices::Ptr indices,
                  Eigen::Vector3f axis, float eps_degrees_tolerance,
                  pcl::ModelCoefficients::Ptr coeff) {
  pcl::PointIndices indices_internal;
  pcl::SACSegmentation<PointC> seg;
  seg.setOptimizeCoefficients(true);

  // Search for a plane perpendicular to some axis (specified below).
  seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);

  // Set the distance to the plane for a point to be a plane inlier.
  seg.setDistanceThreshold(0.01);
  seg.setInputCloud(cloud);

  // Make sure that the plane is perpendicular to given axis, given some degree tolerance.
  seg.setAxis(axis);
  seg.setEpsAngle(pcl::deg2rad(eps_degrees_tolerance));

  // coeff contains the coefficients of the plane:
  // ax + by + cz + d = 0
  pcl::ModelCoefficients plane_coeff;
  seg.segment(indices_internal, plane_coeff);

  *coeff = plane_coeff;
  *indices = indices_internal;
}

void FilterByPlane(pcl::PointCloud<pcl::PointXYZRGB>::Ptr in_cloud,
                   pcl::ModelCoefficients::Ptr plane_coeff,
                   pcl::PointIndices::Ptr plane_side_points) {
  float a = plane_coeff->values[0];
  float b = plane_coeff->values[1];
  float c = plane_coeff->values[2];
  float d = plane_coeff->values[3];
  Eigen::Vector4f plane_abcd = Eigen::Vector4f(a, b, c, d);

  pcl::PlaneClipper3D<PointC> crop = pcl::PlaneClipper3D<PointC>(plane_abcd);
  crop.clipPointCloud3D(*in_cloud, plane_side_points->indices);

  return;
}

Eigen::Vector4f ToHomogeneousCoordsVector(const Eigen::Vector3d in) {
  Eigen::Vector4f out =
      Eigen::Vector4f(static_cast<double>(in[0]), static_cast<double>(in[1]),
                      static_cast<double>(in[2]), 1.0);
  return out;
}

void FilterCloudByIndices(PointCloudC::Ptr in_cloud, PointCloudC::Ptr out_cloud,
                          pcl::PointIndices::Ptr indices, bool invert) {
  pcl::ExtractIndices<PointC> extract;
  extract.setInputCloud(in_cloud);
  extract.setIndices(indices);
  extract.setNegative(invert);
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

void ClusterCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
                  std::vector<pcl::PointIndices>* clusters, double cluster_tolerance,
                  double min_cluster_proportion, double max_cluster_proportion) {
  pcl::EuclideanClusterExtraction<PointC> euclid;
  euclid.setInputCloud(cloud);
  euclid.setClusterTolerance(cluster_tolerance);
  euclid.setMinClusterSize(std::round(min_cluster_proportion * cloud->size()));
  euclid.setMaxClusterSize(std::round(max_cluster_proportion * cloud->size()));
  euclid.extract(*clusters);
}

void GetLargestCluster(std::vector<pcl::PointIndices>* clusters,
                       PointCloudC::Ptr in_cloud, PointCloudC::Ptr out_cloud) {
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
  pcl::ExtractIndices<PointC> handle_extract;

  int clust_size, ind;
  clust_size = 0;
  ind = 0;
  for (size_t i = 0; i < clusters->size(); ++i) {
    if (clusters->at(i).indices.size() > clust_size) {
      ind = i;
    }
  }

  *inliers = clusters->at(ind);
  handle_extract.setInputCloud(in_cloud);
  handle_extract.setIndices(inliers);
  handle_extract.filter(*out_cloud);
}

}  // namespace point_cloud_filtering
