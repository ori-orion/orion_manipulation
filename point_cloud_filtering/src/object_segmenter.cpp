//
// Original author: Mark Finean
// Maintainer: Kim Tien Ly
//

#include "object_segmenter.h"

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

  Eigen::Vector3d search_axis;
  search_axis << 0, 1, 0;
  float eps_degrees_tolerance = 15.0;

  //------ Receive the point cloud --------
  PointCloudC::Ptr cloud(new PointCloudC());
  pcl::fromROSMsg(msg, *cloud);
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
  SegmentPlane(first_cropped_cloud, plane_inliers, search_axis, eps_degrees_tolerance,
               plane_coeff);

  if (plane_inliers->indices.empty()) {
    ROS_ERROR("Could not estimate a planar model for the given dataset.");
    return;
  }

  //------ Calculate and publish the plane parameters and visualisation --------
  Eigen::Vector3d plane_projection;
  CalculatePlaneProjection(plane_coeff, query_point, plane_projection);
  ROS_INFO_STREAM("Calculated plane point:  " << plane_projection);
  Eigen::Isometry3d plane_pose = GetPlanePose(plane_coeff, plane_projection, search_axis);
  PublishPlaneMarker(plane_pose);

  //------ Extract the plane subset of cropped cloud into surface_cloud --------
  PointCloudC::Ptr surface_cloud(new PointCloudC());
  FilterCloudByIndices(first_cropped_cloud, surface_cloud, plane_inliers, false);
  ROS_INFO("Surface cloud has %ld points", surface_cloud->size());

  //------ Extract the non-plane subset of cropped cloud into no_surface_cloud --------
  PointCloudC::Ptr no_surface_cloud(new PointCloudC());
  FilterCloudByIndices(first_cropped_cloud, no_surface_cloud, plane_inliers, true);
  ROS_INFO("Surface-removed cloud has %ld points", no_surface_cloud->size());

  sensor_msgs::PointCloud2 msg_cloud_out;
  pcl::toROSMsg(*no_surface_cloud, msg_cloud_out);
  crop_pub.publish(msg_cloud_out);

  //------ Crop the point cloud using the plane to get the object --------
  // At this point the object cloud may have anything below the surface still in.
  // Use the plane to crop out everything below the surface.
  pcl::PointIndices::Ptr filter_indices(new pcl::PointIndices());
  PointCloudC::Ptr object_cloud(new PointCloudC());
  FilterByPlane(no_surface_cloud, plane_coeff, filter_indices);
  FilterCloudByIndices(no_surface_cloud, object_cloud, filter_indices, false);
  ROS_INFO("Object cloud has %ld points", object_cloud->size());

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
  visual_tools->publishWireframeCuboid(identity_pose, min_crop_pt, max_crop_pt,
                                       rviz_visual_tools::BLUE);
  visual_tools->trigger();
}

void ObjectSegmenter::PublishPlaneMarker(Eigen::Isometry3d plane_pose,
                                         float plane_size,
                                         float arrow_size) {
  // Publish plane and normal vector markers
  visual_tools->publishXYPlane(plane_pose, rviz_visual_tools::BLUE, plane_size);
  visual_tools->publishZArrow(plane_pose, rviz_visual_tools::BLUE,
                              rviz_visual_tools::SMALL, arrow_size);
  visual_tools->trigger();
}

}  // namespace point_cloud_filtering
