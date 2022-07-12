//
// Created by markfinean on 07/05/19.
//

#include "segment_surface_utils.h"
#include "geometry_msgs/PointStamped.h"
#include <eigen_conversions/eigen_msg.h>
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include <visualization_msgs/Marker.h>
#include "geometry_msgs/Point.h"

typedef pcl::PointXYZRGB PointC;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudC;

namespace point_cloud_filtering {

    SurfaceSegmenter::SurfaceSegmenter(const ros::Publisher& object_pub, const double x_in, const double y_in, const double z_in) {
        object_pub_ = object_pub;
        object_x = x_in;
        object_y = y_in;
        object_z = z_in;

        ros::NodeHandle nh;    

        marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 10);
        placeholder_pub = nh.advertise<geometry_msgs::Point>("placeholder", 10);
    }

    void SurfaceSegmenter::Callback(const sensor_msgs::PointCloud2& msg) {

        // Check the incoming point cloud
        PointCloudC::Ptr cloud(new PointCloudC());
        pcl::fromROSMsg(msg, *cloud);
        ROS_INFO("Got point cloud with %ld points", cloud->size());

        visualization_msgs::Marker marker;
        marker.header.frame_id = "head_rgbd_sensor_rgb_frame";
        marker.header.stamp = ros::Time();
        marker.ns = "segmentation_input_point";
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
        ROS_INFO("Marker published");


        //------ Crop the point cloud roughly 20 cm around the object--------
        float crop_radius=0.2;
        Eigen::Vector4f min_crop_pt(object_x-crop_radius,object_y-crop_radius, object_z-crop_radius, 1);
        Eigen::Vector4f max_crop_pt(object_x+crop_radius, object_y+crop_radius, object_z+crop_radius, 1);

        PointCloudC::Ptr first_cropped_cloud(new PointCloudC());
        CropCloud(cloud, first_cropped_cloud, min_crop_pt, max_crop_pt);

        ROS_INFO("Cropped cloud has cloud with %ld points", first_cropped_cloud->size());

        //------ Get the table in the point cloud --------
        pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
        pcl::ModelCoefficients::Ptr coeff(new pcl::ModelCoefficients);
        SegmentPlane(first_cropped_cloud, inliers, coeff);

        // Calculate foot of object on the surface
        float a = coeff->values[0];
        float b = coeff->values[1];
        float c = coeff->values[2];
        float d = coeff->values[3];
        float x1=object_x;
        float y1=object_y;
        float z1=object_z;
        float k = (-a * x1 - b * y1 - c * z1 - d) / (float)(a * a + b * b + c * c);
        float x2 = a * k + x1;
        float y2 = b * k + y1;
        float z2 = c * k + z1;
        ROS_INFO("x= %f", x2);
        ROS_INFO("y= %f", y2);
        ROS_INFO("z= %f", z2);

        // Calculate fripper release location at a predefined distance to the surface 
        float PLACEHOLDER_dist_to_plane = 0.1;
        float tmp = PLACEHOLDER_dist_to_plane/(sqrt(pow((x1-x2),2)+pow((y1-y2),2)+pow((z1-z2),2)));
        float x3 = (x1-x2)*tmp + x2;
        float y3 = (y1-y2)*tmp + y2;
        float z3 = (z1-z2)*tmp + z2;
        ROS_INFO("x3= %f", x3);
        ROS_INFO("y3= %f", y3);
        ROS_INFO("z3= %f", z3);


        if (inliers->indices.empty ())
        {
            PCL_ERROR ("Could not estimate a planar model for the given dataset.");
        }

        geometry_msgs::Point placeholder;
        placeholder.x=x3;
        placeholder.y=y3;
        placeholder.z=z3;
        placeholder_pub.publish(placeholder);

        // Extract the plane indices subset of cloud into output_cloud:
        pcl::ExtractIndices<PointC> table_extract;
        PointCloudC::Ptr table_cloud (new PointCloudC());
        table_extract.setInputCloud(first_cropped_cloud);
        table_extract.setIndices(inliers);
        table_extract.filter(*table_cloud);

        PointCloudC::Ptr surface_cloud(new PointCloudC());
        GetSurface(first_cropped_cloud, surface_cloud, inliers);


        sensor_msgs::PointCloud2 msg_cloud_out;
        pcl::toROSMsg(*surface_cloud, msg_cloud_out);
        object_pub_.publish(msg_cloud_out);


    }

    void SegmentPlane(PointCloudC::Ptr cloud, pcl::PointIndices::Ptr indices, pcl::ModelCoefficients::Ptr coeff) {

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

    void GetSurface(PointCloudC::Ptr in_cloud, PointCloudC::Ptr out_cloud, pcl::PointIndices::Ptr inliers) {
        pcl::ExtractIndices<PointC> extract;
        extract.setInputCloud(in_cloud);
        extract.setIndices(inliers);
        extract.setNegative(false);
        extract.filter(*out_cloud);
    }

    void CropCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr in_cloud,
                   pcl::PointCloud<pcl::PointXYZRGB>::Ptr out_cloud,
                   Eigen::Vector4f min_p,
                   Eigen::Vector4f max_p){

        pcl::CropBox<PointC> crop;
        crop.setInputCloud(in_cloud);
        crop.setMin(min_p);
        crop.setMax(max_p);
        crop.filter(*out_cloud);
    }



} //namespace point_cloud_filtering
