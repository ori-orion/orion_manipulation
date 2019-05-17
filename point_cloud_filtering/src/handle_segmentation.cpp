#include "handle_segmentation.h"
#include "handle_utils.h"
#include "geometry_msgs/PointStamped.h"
#include <eigen_conversions/eigen_msg.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree_flann.h>

#include "sensor_msgs/JointState.h"

namespace point_cloud_filtering {

    HandleCropper::HandleCropper(const ros::Publisher& cloud_pub, const ros::Publisher& door_pub ) : cloud_pub_(cloud_pub), door_pub_(door_pub) {}

    HandleCentroid::HandleCentroid(const tf::TransformBroadcaster& br) : handle_tf_br_(br), good_detection_(false) {}

    DrawerHandleCropper::DrawerHandleCropper(const ros::Publisher& cloud_pub, const ros::Publisher& plane_pub ) : cloud_pub_(cloud_pub), plane_pub_(plane_pub) {}

    DrawerHandleCentroid::DrawerHandleCentroid(const tf::TransformBroadcaster& br) : handle_tf_br_(br), good_detection_(false) {}

    //    Doors
    void HandleCropper::Callback(const sensor_msgs::PointCloud2& msg) {

      // Check the incoming point cloud
      PointCloudC::Ptr cloud(new PointCloudC());
      pcl::fromROSMsg(msg, *cloud);
      ROS_INFO("Got point cloud with %ld points", cloud->size());

        //------ Crop the point cloud used to get the handle --------
        Eigen::Vector4f min_crop_pt(-0.4, -0.5, 0, 1);
        Eigen::Vector4f max_crop_pt(0.4, 0.5, 2, 1);
        PointCloudC::Ptr first_cropped_cloud(new PointCloudC());
        CropCloud(cloud, first_cropped_cloud, min_crop_pt, max_crop_pt);


      //------ Get the door --------
      pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
      SegmentDoorInliers(first_cropped_cloud, inliers);

      if (inliers->indices.empty ())
      {
        PCL_ERROR ("Could not estimate a planar model for the given dataset.");
      }

      // Extract the plane indices subset of cloud into output_cloud:
      pcl::ExtractIndices<PointC> door_extract;
      PointCloudC::Ptr door_cloud (new PointCloudC());
      door_extract.setInputCloud(first_cropped_cloud);
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
      RemoveDoor(first_cropped_cloud, filtered_cloud, inliers);

      ROS_INFO("Min z: %ld", min_z);
      ROS_INFO("Max z: %ld", max_z);

      //------ Crop the point cloud used to get the handle --------
      Eigen::Vector4f min_pt(min_x+0.13, min_y+0.13, min_z-0.12, 1);
//      Eigen::Vector4f max_pt(max_x-0.13, max_y-0.13, max_z-0.06, 1);
      Eigen::Vector4f max_pt(max_x-0.13, max_y-0.13, min_z-0.02, 1);
      PointCloudC::Ptr cropped_cloud(new PointCloudC());
      CropCloud(filtered_cloud, cropped_cloud, min_pt, max_pt);

      // Publish the handle point cloud
      sensor_msgs::PointCloud2 msg_cloud_out;
      pcl::toROSMsg(*cropped_cloud, msg_cloud_out);
      cloud_pub_.publish(msg_cloud_out);

      // Publish the door point cloud
      sensor_msgs::PointCloud2 msg_door_cloud_out;
      pcl::toROSMsg(*door_cloud, msg_door_cloud_out);
      door_pub_.publish(msg_door_cloud_out);

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

    //    Drawer

    void DrawerHandleCropper::GetHeadAngle(const sensor_msgs::JointState& msg) {
        head_angle = msg.position[13];
    }

    void DrawerHandleCropper::Callback(const sensor_msgs::PointCloud2& msg) {

        // Check the incoming point cloud
        PointCloudC::Ptr cloud(new PointCloudC());
        pcl::fromROSMsg(msg, *cloud);
        ROS_INFO("Got point cloud with %ld points", cloud->size());


        //------ Crop the point cloud used to get the handle --------
        // z in direction of looking, x to right, y perpendicular (down and back)
        Eigen::Vector4f min_crop_pt(-0.2, 0.0, 0, 1);
        Eigen::Vector4f max_crop_pt(0.2, 1.0, 2, 1);
        PointCloudC::Ptr first_cropped_cloud(new PointCloudC());
        CropCloud(cloud, first_cropped_cloud, min_crop_pt, max_crop_pt);
        ROS_INFO("Initial crop leaves point cloud with %ld points", first_cropped_cloud->size());


        //------ Get the door --------
        Eigen::Vector3f axis;
        axis << 0, -sin(head_angle), cos(head_angle);

        std::cout << axis << std::endl;
        pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
        SegmentFurnitureInliers(first_cropped_cloud, inliers, coefficients, axis);

        if (inliers->indices.empty ())
        {
            PCL_ERROR ("Could not estimate a planar model for the given dataset.");
        }

        // Extract the plane indices subset of cloud into output_cloud:
        pcl::ExtractIndices<PointC> door_extract;
        PointCloudC::Ptr door_cloud (new PointCloudC());
        door_extract.setInputCloud(first_cropped_cloud);
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

        ROS_INFO("Min x: %f", min_x);
        ROS_INFO("Max x: %f", max_x);
        ROS_INFO("Min y: %f", min_y);
        ROS_INFO("Max y: %f", max_y);
        ROS_INFO("Min z: %f", min_z);
        ROS_INFO("Max z: %f", max_z);

        // Remove the door
        PointCloudC::Ptr filtered_cloud(new PointCloudC());
        RemoveDoor(first_cropped_cloud, filtered_cloud, inliers);


        //------ Crop the point cloud used to get the handle --------
        Eigen::Vector4f min_pt(min_x + 0.1,
                               min_y,
                               min_z,
                               1);

        Eigen::Vector4f max_pt(max_x - 0.1, max_y - 0.1 * axis[2], max_z-0.1*axis[1], 1);

        PointCloudC::Ptr handle_cloud(new PointCloudC());
        CropCloud(filtered_cloud, handle_cloud, min_pt, max_pt);

        // Publish the plane point cloud
        sensor_msgs::PointCloud2 msg_door_cloud_out;
        pcl::toROSMsg(*door_cloud, msg_door_cloud_out);
        plane_pub_.publish(msg_door_cloud_out);

        // Publish the handle point cloud
        sensor_msgs::PointCloud2 msg_cloud_out;
        pcl::toROSMsg(*handle_cloud, msg_cloud_out);
        cloud_pub_.publish(msg_cloud_out);

    }

    void DrawerHandleCentroid::Callback(const sensor_msgs::PointCloud2& msg) {

        // Check the incoming point cloud
        PointCloudC::Ptr handle_cloud(new PointCloudC());
        pcl::fromROSMsg(msg, *handle_cloud);
        ROS_INFO("Got point cloud with %ld points", handle_cloud->size());

        // At this point we may have multiple handles detected
        std::vector<pcl::PointIndices>* clusters;
        GetClusters(handle_cloud, clusters);

        for(size_t i=0; i < clusters->size(); ++i) {

            pcl::PointIndices::Ptr handle_inliers(new pcl::PointIndices());
            pcl::ExtractIndices<PointC> handle_extract;
            PointCloudC::Ptr clustered_handle_cloud(new PointCloudC());

            *handle_inliers = clusters->at(i);
            handle_extract.setInputCloud(handle_cloud);
            handle_extract.setIndices(handle_inliers);
            handle_extract.filter(*clustered_handle_cloud);

            // publish centroid
            Eigen::Vector4f centroid;
            pcl::compute3DCentroid(*handle_cloud, centroid);
            std::cout << "The centroid is: " << std::endl;
            std::cout << "x:" << centroid[0] << " y:" << centroid[1] << "z: " << centroid[2] << std::endl;

            float x, y, z;

            x = centroid[0];
            y = centroid[1];
            z = centroid[2];

            tf::Transform transform;
            transform.setOrigin(tf::Vector3(x, y, z));
            transform.setRotation(tf::Quaternion(0, 0, 0));

            std::stringstream ss;
            ss << i;

            std::string drawer_handle_name = "drawer_handle_" + ss.str();

            handle_tf_br_.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "head_rgbd_sensor_rgb_frame",
                                                             drawer_handle_name));


            //  If the pose seems reasonable then store and prepare to exit the service
            if (z > 0.2 && z < 1.2 && x > -0.5 && x < 0.5 && y > -0.5 && y < 0.5 && DrawerHandleCentroid::good_detection_ == false) {
                DrawerHandleCentroid::good_detection_ = true;
                DrawerHandleCentroid::x_ = x;
                DrawerHandleCentroid::y_ = y;
                DrawerHandleCentroid::z_ = z;
                std::cout << "Criteria matched!" << std::endl;
            }

        }

    }

    bool DrawerHandleCentroid::CheckDetection() {
        return DrawerHandleCentroid::good_detection_;
    }

    double DrawerHandleCentroid::GetX(){
        return DrawerHandleCentroid::x_;
    }

    double DrawerHandleCentroid::GetY(){
        return DrawerHandleCentroid::y_;
    }

    double DrawerHandleCentroid::GetZ(){
        return DrawerHandleCentroid::z_;
    }

} //namespace point_cloud_filtering
