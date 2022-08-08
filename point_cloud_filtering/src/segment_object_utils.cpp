//
// Original author: Mark Finean
// Maintainer: Kim Tien Ly
//

#include "segment_object_utils.h"

#include <eigen_conversions/eigen_msg.h>
#include <visualization_msgs/Marker.h>

#include "geometry_msgs/Point.h"
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"

typedef pcl::PointXYZRGB PointC;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudC;

namespace point_cloud_filtering {

ObjectSegmenter::ObjectSegmenter(const ros::Publisher& object_pub, const double x_in,
                                 const double y_in, const double z_in) {
  object_pub_ = object_pub;
  query_point << x_in, y_in, z_in;

  ros::NodeHandle nh;

  // Specific to object segmentation
  crop_pub = nh.advertise<sensor_msgs::PointCloud2>("cropped_cloud", 1, true);

  visual_tools.reset(new rviz_visual_tools::RvizVisualTools("head_rgbd_sensor_rgb_frame",
                                                            "vis_markers"));
  visual_tools->trigger();
}

void ObjectSegmenter::Callback(const sensor_msgs::PointCloud2& msg) {
  //------ Receive the point cloud --------
  PointCloudC::Ptr cloud(new PointCloudC());
  pcl::fromROSMsg(msg, *cloud);
  ROS_INFO("Got point cloud with %ld points", cloud->size());

  //------ Reset any visualisation --------
  visual_tools->deleteAllMarkers();

  //------ Crop the point cloud roughly 15 cm around the target --------
  float crop_radius = 0.15;
  Eigen::Vector3d min_crop_pt = query_point.array() - crop_radius;
  Eigen::Vector3d max_crop_pt = query_point.array() + crop_radius;
  ROS_INFO_STREAM("segment_utils: Crop points:  " << min_crop_pt << ", " << max_crop_pt);
  PublishCropBoundingBoxMarker(min_crop_pt, max_crop_pt);

  PointCloudC::Ptr first_cropped_cloud(new PointCloudC());
  CropCloud(cloud, first_cropped_cloud, ToHomogeneousCoordsVector(min_crop_pt),
            ToHomogeneousCoordsVector(max_crop_pt));

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

  //------ Calculate and publish the plane parameters and visualisation --------
  Eigen::Vector3d plane_projection;
  CalculatePlaneProjection(plane_coeff, query_point, plane_projection);
  ROS_INFO_STREAM("segment_utils: Calculated plane point:  " << plane_projection);
  PublishPlaneMarker(plane_coeff, plane_projection);

  //------ Extract the plane subset of cropped cloud into surface_cloud --------
  PointCloudC::Ptr surface_cloud(new PointCloudC());
  FilterCloudInliers(first_cropped_cloud, surface_cloud, inliers, false);

  //------ Extract the non-plane subset of cropped cloud into no_surface_cloud --------
  PointCloudC::Ptr no_surface_cloud(new PointCloudC());
  FilterCloudInliers(first_cropped_cloud, no_surface_cloud, inliers, true);

  sensor_msgs::PointCloud2 msg_cloud_out;
  pcl::toROSMsg(*no_surface_cloud, msg_cloud_out);
  ROS_INFO("Publishing no_surface_cloud");
  crop_pub.publish(msg_cloud_out);

  // At this point the object cloud may have anything below the table still in.
  // The y axis points towards the floor so need to crop anything above max y value in the
  // table inliers.

  float min_y, max_y;
  GetCloudMinMaxY(surface_cloud, min_y, max_y);
  ROS_INFO("min_y= %f ", min_y);
  ROS_INFO("max_y= %f ", max_y);

  //------ Crop the point cloud used to get the object --------
  max_crop_pt[1] = min_y;
  PointCloudC::Ptr object_cloud(new PointCloudC());
  CropCloud(no_surface_cloud, object_cloud, ToHomogeneousCoordsVector(min_crop_pt),
            ToHomogeneousCoordsVector(max_crop_pt));
  ROS_INFO_STREAM("segment_utils: Crop points:  " << min_crop_pt << ", " << max_crop_pt);
  PublishCropBoundingBoxMarker(min_crop_pt, max_crop_pt);

  //------ Publish the object point cloud --------
  pcl::toROSMsg(*object_cloud, msg_cloud_out);
  object_pub_.publish(msg_cloud_out);
}

void ObjectSegmenter::CalculatePlaneProjection(pcl::ModelCoefficients::Ptr plane_coeff,
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

void ObjectSegmenter::PublishCropBoundingBoxMarker(Eigen::Vector3d min_crop_pt,
                                                   Eigen::Vector3d max_crop_pt) {
  Eigen::Isometry3d identity_pose = Eigen::Isometry3d::Identity();

  ROS_INFO("Publishing bounding box marker");
  visual_tools->publishWireframeCuboid(identity_pose, min_crop_pt, max_crop_pt,
                                       rviz_visual_tools::BLUE);
  visual_tools->trigger();
}

void ObjectSegmenter::PublishPlaneMarker(pcl::ModelCoefficients::Ptr plane_coeff,
                                         Eigen::Vector3d plane_projection) {
  // Publish plane and normal vector markers
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

void FilterCloudInliers(PointCloudC::Ptr in_cloud, PointCloudC::Ptr out_cloud,
                        pcl::PointIndices::Ptr inliers, bool invert) {
  pcl::ExtractIndices<PointC> extract;
  extract.setInputCloud(in_cloud);
  extract.setIndices(inliers);
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

Eigen::Vector4f ToHomogeneousCoordsVector(const Eigen::Vector3d in) {
  Eigen::Vector4f out =
      Eigen::Vector4f(static_cast<double>(in[0]), static_cast<double>(in[1]),
                      static_cast<double>(in[2]), 1.0);
  return out;
}

}  // namespace point_cloud_filtering
