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

ObjectSegmenter::ObjectSegmenter(ros::NodeHandle* nh) : SurfaceSegmenter(nh) {
  crop_point_cloud_pub =
      nh->advertise<sensor_msgs::PointCloud2>("cropped_cloud", 1, true);

  object_point_cloud_pub =
      nh->advertise<sensor_msgs::PointCloud2>("object_cloud", 1, true);
}

void ObjectSegmenter::StartServices(void) {
  service_server = ros_node_handle->advertiseService(
      "segment_object", &ObjectSegmenter::ServiceCallback, this);
  ROS_INFO("%s: /segment_object service ready", ros::this_node::getName().c_str());
}

bool ObjectSegmenter::ServiceCallback(
    point_cloud_filtering::SegmentObject::Request& req,
    point_cloud_filtering::SegmentObject::Response& res) {
  float eps_degrees_tolerance = req.eps_degrees_tolerance;
  float crop_box_dimension = req.search_box_dimension;

  //------ Reset any visualisation --------
  visual_tools->deleteAllMarkers();
  PublishTransformMarker(req.search_axis);

  // Plane search point transform translation
  Eigen::Vector3d query_point;
  query_point << req.search_axis.translation.x, req.search_axis.translation.y,
      req.search_axis.translation.z;

  // Plane search axis from Z-direction vector given by transform
  Eigen::Quaterniond rotQ;
  tf::quaternionMsgToEigen(req.search_axis.rotation, rotQ);
  Eigen::Quaterniond z;
  z.w() = 0;
  z.vec() = Eigen::Vector3d::UnitZ();
  Eigen::Quaterniond rotatedP = rotQ * z * rotQ.inverse();
  Eigen::Vector3d search_axis = rotatedP.vec();

  res.success = false;

  //------ Receive one point cloud from RGBD sensor --------
  sensor_msgs::PointCloud2ConstPtr msg =
      ros::topic::waitForMessage<sensor_msgs::PointCloud2>("cloud_in", ros::Duration(3));
  if (msg == NULL) {
    ROS_ERROR("%s: no point cloud received in timeout", node_name);
    return false;
  }

  PointCloudC::Ptr cloud(new PointCloudC());
  pcl::fromROSMsg(*msg, *cloud);
  ROS_INFO("%s: received point cloud with %ld points", node_name, cloud->size());

  //------ Crop the point cloud roughly 15 cm around the target --------
  Eigen::Vector3d min_crop_pt = query_point.array() - crop_box_dimension;
  Eigen::Vector3d max_crop_pt = query_point.array() + crop_box_dimension;
  ROS_INFO_STREAM(node_name << ": crop points:  " << min_crop_pt << ", " << max_crop_pt);
  PublishCropBoundingBoxMarker(min_crop_pt, max_crop_pt);

  PointCloudC::Ptr first_cropped_cloud(new PointCloudC());
  CropCloud(cloud, first_cropped_cloud, ToHomogeneousCoordsVector(min_crop_pt),
            ToHomogeneousCoordsVector(max_crop_pt));

  ROS_INFO("%s: box cropped cloud has %ld points", node_name,
           first_cropped_cloud->size());

  //------ Get the surface in the point cloud --------
  pcl::PointIndices::Ptr plane_inliers(new pcl::PointIndices());
  pcl::ModelCoefficients::Ptr plane_coeff(new pcl::ModelCoefficients);

  SegmentPlane(first_cropped_cloud, plane_inliers, search_axis, eps_degrees_tolerance,
               plane_coeff);

  if (plane_inliers->indices.empty()) {
    ROS_ERROR("%s: could not estimate a planar model for the given dataset.", node_name);
    return false;
  }

  //------ Extract the plane subset of cropped cloud into surface_cloud --------
  PointCloudC::Ptr surface_cloud(new PointCloudC());
  FilterCloudByIndices(first_cropped_cloud, surface_cloud, plane_inliers, false);
  ROS_INFO("%s: surface cloud has %ld points", node_name, surface_cloud->size());

  sensor_msgs::PointCloud2 msg_cloud_out;
  pcl::toROSMsg(*surface_cloud, msg_cloud_out);
  surface_point_cloud_pub.publish(msg_cloud_out);

  //------ Extract the non-plane subset of cropped cloud into no_surface_cloud --------
  PointCloudC::Ptr no_surface_cloud(new PointCloudC());
  FilterCloudByIndices(first_cropped_cloud, no_surface_cloud, plane_inliers, true);
  ROS_INFO("Surface-removed cloud has %ld points", no_surface_cloud->size());

  pcl::toROSMsg(*no_surface_cloud, msg_cloud_out);
  crop_point_cloud_pub.publish(msg_cloud_out);

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
  object_point_cloud_pub.publish(msg_cloud_out);

  //------ Calculate the plane transform, show visualisation and return transform --------
  Eigen::Vector3d plane_projection;
  CalculatePlaneProjection(plane_coeff, query_point, plane_projection);

  ROS_INFO_STREAM(node_name << ": calculated plane point:  " << plane_projection);
  ROS_INFO_STREAM(node_name << ": plane parameters:  " << *plane_coeff);
  Eigen::Isometry3d plane_pose = GetPlanePose(plane_coeff, plane_projection, search_axis);
  PublishPlaneMarker(plane_pose, crop_box_dimension);

  geometry_msgs::Transform transform_msg;
  tf::transformEigenToMsg(plane_pose, transform_msg);
  res.plane_axis = transform_msg;

  res.success = true;
  return true;
}

}  // namespace point_cloud_filtering
