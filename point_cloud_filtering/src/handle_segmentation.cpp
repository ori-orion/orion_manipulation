#include "handle_segmentation.h"
#include "handle_utils.h"
#include "geometry_msgs/PointStamped.h"
#include <eigen_conversions/eigen_msg.h>

namespace point_cloud_filtering {

    HandleCropper::HandleCropper(const ros::Publisher& cloud_pub, const ros::Publisher& handle_centroid_pub) : cloud_pub_(cloud_pub), handle_centroid_pub_(handle_centroid_pub) {}

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
//      Eigen::Vector4f max_pt(max_x-0.13, max_y-0.1, max_z+0.05, 1);
      Eigen::Vector4f max_pt(max_x-0.13, max_y-0.1, max_z-0.04, 1);
      PointCloudC::Ptr cropped_cloud(new PointCloudC());
      CropCloud(filtered_cloud, cropped_cloud, min_pt, max_pt);

      // Extract the handle indices
//      std::vector<pcl::PointIndices> protruding_clusters;
//      GetClusters(cropped_cloud, &protruding_clusters);
//      std::cout << "Door Clusters succeeded." << std::endl;

//      ROS_INFO("There are %ld clusters", protruding_clusters.size());

//      std::cout << "No clusters detected. Publishing door" << std::endl;
      Eigen::Vector4f centroid;
      pcl::compute3DCentroid(*cropped_cloud, centroid);
      std::cout << "The centroid is: " << std::endl;
      std::cout << "x:" << centroid[0] << " y:" << centroid[1] << "z: " << centroid[2] << std::endl;

      // Publish the handle point cloud and centroid
      sensor_msgs::PointCloud2 msg_cloud_out;
//      geometry_msgs::Vector3Stamped msg_handle_centroid_out;

      pcl::toROSMsg(*cropped_cloud, msg_cloud_out);
//      tf::vectorEigenToMsg(Eigen::Vector3f(centroid[0], centroid[1], centroid[2]), msg_handle_centroid_out)

      cloud_pub_.publish(msg_cloud_out);
//      handle_centroid_pub_.publish(msg_handle_centroid_out);

      // publish centroid
      geometry_msgs::PointStamped msg_handle_centroid_out;
      msg_handle_centroid_out.header.frame_id = "head_rgbd_sensor_rgb_frame";
      msg_handle_centroid_out.header.stamp = ros::Time();
      msg_handle_centroid_out.point.x = centroid[0];
      msg_handle_centroid_out.point.y = centroid[1];
      msg_handle_centroid_out.point.z = centroid[2];
//      geometry_msgs::PointStamped msg_handle_centroid_out;
//      listener->transformPoint("torso", out_pos, msg_handle_centroid_out);
        handle_centroid_pub_.publish(msg_handle_centroid_out);

//      if(not protruding_clusters.empty() > 0){
//         PointCloudC::Ptr handle_cloud(new PointCloudC);
//         GetHandleFromClusters(&protruding_clusters, cropped_cloud, handle_cloud);
//
//         Eigen::Vector4f centroid;
//         pcl::compute3DCentroid(*handle_cloud, centroid);
//
//         std::cout << "The centroid is: " << centroid[0] << " " << centroid[1] << " " << centroid[2] << std::endl;
//         // Publish output cloud
//         sensor_msgs::PointCloud2 msg_out;
//         pcl::toROSMsg(*handle_cloud, msg_out);
//         pub_.publish(msg_out);
//       }
//      else{
//         std::cout << "No clusters detected. Publishing door" << std::endl;
//         sensor_msgs::PointCloud2 msg_out;
//         pcl::toROSMsg(*cropped_cloud, msg_out);
//         pub_.publish(msg_out);
//      }
    }

} //namespace point_cloud_filtering
