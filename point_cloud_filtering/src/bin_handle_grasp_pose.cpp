//
// Created by Mark Finean on 2019-07-04.
//

#include "bin_handle_segmentation.h"
#include "handle_utils.h"
#include "geometry_msgs/PointStamped.h"
#include "point_cloud_filtering/DetectBinHandle.h"
#include <ros/callback_queue.h>

#include <ctime>
#include <iostream>
#include <cstdio>

bool detect_bin_handle(point_cloud_filtering::DetectBinHandle::Request  &req,
                   point_cloud_filtering::DetectBinHandle::Response &res){

    ROS_INFO("Message request received.");
    ros::NodeHandle nh;
    tf::TransformBroadcaster br;

    // To publish cropped door and handle
    ros::Publisher bin_handle_pub =
            nh.advertise<sensor_msgs::PointCloud2>("bin_handle_cloud", 1, true);

    ros::Publisher bin_surface_pub =
            nh.advertise<sensor_msgs::PointCloud2>("bin_surface_cloud", 1, true);

    // Crop for the handle
    point_cloud_filtering::BinHandleCropper bin_handle_cropper(bin_handle_pub, bin_surface_pub);
    point_cloud_filtering::BinHandleCentroid bin_handle_centroid(br);

    ros::Subscriber sub_handle =
            nh.subscribe("cloud_in", 1, &point_cloud_filtering::BinHandleCropper::Callback,  &bin_handle_cropper);

    ros::Subscriber sub_handle_centroid =
            nh.subscribe("bin_handle_cloud", 1, &point_cloud_filtering::BinHandleCentroid::Callback,  &bin_handle_centroid);

    // Until we get a good validated detection of the handle keep looping
    // Need to insert a timer here using argument from high level

    std::clock_t start;
    double duration = 0;
    start = std::clock();

    while ((not bin_handle_centroid.CheckDetection()) and duration < 10)
    {
        ros::getGlobalCallbackQueue()->callAvailable(ros::WallDuration(0.1));
        duration = ( std::clock() - start ) / (double) CLOCKS_PER_SEC;
    }

    if (not bin_handle_centroid.CheckDetection()){
        res.x = 0;
        res.y = 0;
        res.z = 0;
        res.handle_detected = false;
        return false;
    }
    else {
        res.x = bin_handle_centroid.GetX();
        res.y = bin_handle_centroid.GetY();
        res.z = bin_handle_centroid.GetZ();
        res.handle_detected = true;
        return true;
    }




}

int main(int argc, char** argv) {
    ros::init(argc, argv, "bin_handle_grasp_pose");
    ros::NodeHandle nh;

    ros::ServiceServer service = nh.advertiseService("bin_handle_detection", detect_bin_handle);

    ROS_INFO("Ready to detect bin handles.");

    ros::spin();

    return 0;
}
