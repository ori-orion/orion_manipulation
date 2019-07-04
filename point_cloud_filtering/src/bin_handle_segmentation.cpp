//
// Created by Mark Finean on 2019-05-16.
//

#include "bin_handle_segmentation.h"
#include "handle_utils.h"
#include "geometry_msgs/PointStamped.h"
#include <eigen_conversions/eigen_msg.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree_flann.h>

#include "sensor_msgs/JointState.h"

namespace point_cloud_filtering {

    BinHandleCropper::BinHandleCropper(const ros::Publisher& bin_handle_pub, const ros::Publisher& bin_surface_pub ) : bin_handle_pub_(bin_handle_pub), bin_surface_pub_(bin_surface_pub) {}

    BinHandleCentroid::BinHandleCentroid(const tf::TransformBroadcaster& br) : bin_handle_tf_br_(br), good_detection_(false) {}

    //    Doors
    void BinHandleCropper::Callback(const sensor_msgs::PointCloud2& msg) {

        // Check the incoming point cloud
        PointCloudC::Ptr cloud(new PointCloudC());
        pcl::fromROSMsg(msg, *cloud);
        ROS_INFO("Got point cloud with %ld points", cloud->size());

        //------ Crop the point cloud used to get the handle --------
        Eigen::Vector4f min_crop_pt(-0.4, 0, 0, 1);
        Eigen::Vector4f max_crop_pt(0.4, 1, 1, 1);
        PointCloudC::Ptr first_cropped_cloud(new PointCloudC());
        CropCloud(cloud, first_cropped_cloud, min_crop_pt, max_crop_pt);


        //------ Get the door --------
        pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
        SegmentBinSurfaceInliers(first_cropped_cloud, inliers);

        if (inliers->indices.empty ())
        {
            PCL_ERROR ("Could not estimate a planar model for the given dataset.");
        }

        // Extract the plane indices subset of cloud into output_cloud:
        pcl::ExtractIndices<PointC> bin_surface_extract;
        PointCloudC::Ptr bin_cloud (new PointCloudC());
        bin_surface_extract.setInputCloud(first_cropped_cloud);
        bin_surface_extract.setIndices(inliers);
        bin_surface_extract.filter(*bin_cloud);


        float min_x = std::numeric_limits<float>::max();
        float min_y = std::numeric_limits<float>::max();
        float min_z = std::numeric_limits<float>::max();
        float max_x = std::numeric_limits<float>::min();
        float max_y = std::numeric_limits<float>::min();
        float max_z = std::numeric_limits<float>::min();

        for(size_t i=0; i < bin_cloud->points.size(); ++i) {
            if (bin_cloud->points[i].x < min_x) {
                min_x = bin_cloud->points[i].x;
            }
            if (bin_cloud->points[i].y < min_y) {
                min_y = bin_cloud->points[i].y;
            }
            if (bin_cloud->points[i].z < min_z) {
                min_z = bin_cloud->points[i].z;
            }
            if (bin_cloud->points[i].x > max_x) {
                max_x = bin_cloud->points[i].x;
            }
            if (bin_cloud->points[i].y > max_y) {
                max_y = bin_cloud->points[i].y;
            }
            if (bin_cloud->points[i].z > max_z) {
                max_z = bin_cloud->points[i].z;
            }
        }

        // Remove the door
//        PointCloudC::Ptr filtered_cloud(new PointCloudC());
//        RemoveDoor(first_cropped_cloud_cloud, filtered_cloud, inliers);

        ROS_INFO("Min z: %ld", min_z);
        ROS_INFO("Max z: %ld", max_z);

        //------ Crop the point cloud used to get the handle --------
        Eigen::Vector4f min_pt(min_x, 0, min_z,1);
        Eigen::Vector4f max_pt(max_x, min_y, min_z, 1);
        PointCloudC::Ptr cropped_cloud(new PointCloudC());
        CropCloud(cloud, cropped_cloud, min_pt, max_pt);

        // Publish the handle point cloud
        sensor_msgs::PointCloud2 msg_cloud_out;
        pcl::toROSMsg(*cropped_cloud, msg_cloud_out);
        bin_handle_pub_.publish(msg_cloud_out);

        // Publish the door point cloud
        sensor_msgs::PointCloud2 msg_bin_surface_cloud_out;
        pcl::toROSMsg(*bin_cloud, msg_bin_surface_cloud_out);
        bin_surface_pub_.publish(msg_bin_surface_cloud_out);

    }

    void BinHandleCentroid::Callback(const sensor_msgs::PointCloud2& msg) {

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

        bin_handle_tf_br_.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "head_rgbd_sensor_rgb_frame", "bin_handle"));

        //  If the pose seems reasonable then store and prepare to exit the service
        if(z>0.2 && z<2 && x>-0.5 && x<0.5 && y>0 && y<2){
            BinHandleCentroid::good_detection_ = true;
            BinHandleCentroid::x_ = x;
            BinHandleCentroid::y_ = y;
            BinHandleCentroid::z_ = z;
            std::cout << "Criteria matched!" << std::endl;
        }

    }

    bool BinHandleCentroid::CheckDetection() {
        return BinHandleCentroid::good_detection_;
    }

    double BinHandleCentroid::GetX(){
        return BinHandleCentroid::x_;
    }

    double BinHandleCentroid::GetY(){
        return BinHandleCentroid::y_;
    }

    double BinHandleCentroid::GetZ(){
        return BinHandleCentroid::z_;
    }

} //namespace point_cloud_filtering
