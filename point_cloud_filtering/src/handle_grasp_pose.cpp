//
// Created by Mark Finean on 2019-05-16.
//

#include "handle_segmentation.h"
#include "handle_utils.h"
#include "geometry_msgs/PointStamped.h"
#include "point_cloud_filtering/DetectHandle.h"
#include <ros/callback_queue.h>

#include <ctime>
#include <iostream>
#include <cstdio>

bool detect_handle(point_cloud_filtering::DetectHandle::Request  &req,
                   point_cloud_filtering::DetectHandle::Response &res){

    ROS_INFO("Handle detection message received by server.");

    ros::NodeHandle nh;
    tf::TransformBroadcaster br;

    // To publish cropped door and handle
    ros::Publisher handle_pub =
            nh.advertise<sensor_msgs::PointCloud2>("handle_cloud", 1, true);

    ros::Publisher door_pub =
            nh.advertise<sensor_msgs::PointCloud2>("door_cloud", 1, true);

    // Crop for the handle
    point_cloud_filtering::HandleCropper handle_cropper(handle_pub, door_pub);
    point_cloud_filtering::HandleCentroid handle_centroid(br);

    ros::Subscriber sub_handle =
            nh.subscribe("cloud_in", 1, &point_cloud_filtering::HandleCropper::Callback,  &handle_cropper);

    ros::Subscriber sub_handle_centroid =
            nh.subscribe("handle_cloud", 1, &point_cloud_filtering::HandleCentroid::Callback,  &handle_centroid);

    // Until we get a good validated detection of the handle keep looping
    // Need to insert a timer here using argument from high level

    std::clock_t start;
    double duration = 0;
    start = std::clock();

    while ((not handle_centroid.CheckDetection()) and duration < 10)
    {
        ROS_INFO("Trying to get a good detection.");
        ros::getGlobalCallbackQueue()->callAvailable(ros::WallDuration(0.1));
        duration = ( std::clock() - start ) / (double) CLOCKS_PER_SEC;
    }

    if (not handle_centroid.CheckDetection()){
        ROS_INFO("No good detection found.");
        res.x = 0;
        res.y = 0;
        res.z = 0;
        res.handle_detected = false;
        return false;
    }
    else {
        ROS_INFO("Handle detected and returning values.");
        res.x = handle_centroid.GetX();
        res.y = handle_centroid.GetY();
        res.z = handle_centroid.GetZ();
        res.handle_detected = true;
        return true;
    }




}

int main(int argc, char** argv) {
  ros::init(argc, argv, "handle_grasp_pose");
  ros::NodeHandle nh;

  ros::ServiceServer service = nh.advertiseService("handle_detection", detect_handle);

  ROS_INFO("Ready to detect handles.");

  ros::spin();

  return 0;
}
