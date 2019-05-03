#include "handle_segmentation.h"
#include "handle_utils.h"
#include "geometry_msgs/PointStamped.h"
#include <eigen_conversions/eigen_msg.h>

namespace point_cloud_filtering {

    HandleCropper::HandleCropper(const ros::Publisher& cloud_pub) : cloud_pub_(cloud_pub) {}

    HandleCentroid::HandleCentroid(const tf::TransformBroadcaster& br) : handle_tf_br_(br), good_detection_(false) {}

    void HandleCropper::Callback(const sensor_msgs::PointCloud2& msg) {

      // Check the incoming point cloud
      PointCloudC::Ptr cloud(new PointCloudC());
      pcl::fromROSMsg(msg, *cloud);
      ROS_INFO("Got point cloud with %ld points", cloud->size());

      //------ Get the door --------
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
      Eigen::Vector4f max_pt(max_x-0.13, max_y-0.1, max_z-0.04, 1);
      PointCloudC::Ptr cropped_cloud(new PointCloudC());
      CropCloud(filtered_cloud, cropped_cloud, min_pt, max_pt);

      // Publish the handle point cloud and centroid
      sensor_msgs::PointCloud2 msg_cloud_out;
      pcl::toROSMsg(*cropped_cloud, msg_cloud_out);
      cloud_pub_.publish(msg_cloud_out);

    }

    bool HandleCentroid::CheckDetection() {
        return HandleCentroid::good_detection_;
    }

    double HandleCentroid::GetX(){
        return HandleCentroid::x_;
    }
    double HandleCentroid::GetY(){
        return HandleCentroid::y_;
    }

    double HandleCentroid::GetZ(){
        return HandleCentroid::z_;
    }

    void HandleCentroid::Callback(const sensor_msgs::PointCloud2& msg) {

        // Check the incoming point cloud
        PointCloudC::Ptr handle_cloud(new PointCloudC());
        pcl::fromROSMsg(msg, *handle_cloud);
        ROS_INFO("Got point cloud with %ld points", handle_cloud->size());

        // publish centroid
        Eigen::Vector4f centroid;
        pcl::compute3DCentroid(*handle_cloud, centroid);
        std::cout << "The centroid is: " << std::endl;
        std::cout << "x:" << centroid[0] << " y:" << centroid[1] << "z: " << centroid[2] << std::endl;

        float x,y,z;

        x = centroid[0];
        y = centroid[1];
        z = centroid[2];

        tf::Transform transform;
        transform.setOrigin(tf::Vector3(x, y, z));
        transform.setRotation(tf::Quaternion(0, 0, 0));

        handle_tf_br_.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "head_rgbd_sensor_rgb_frame", "door_handle"));

        //  If the pose seems reasonable then store and prepare to exit the service
        if(z>0.3 && z<1.2 && x>-0.5 && x<0.5 && y>-0.5 && y<0.5){
            HandleCentroid::good_detection_ = true;
            HandleCentroid::x_ = x;
            HandleCentroid::y_ = y;
            HandleCentroid::z_ = z;
            std::cout << "Criteria matched!" << std::endl;
        }

    }

} //namespace point_cloud_filtering
