#include "handle_segmentation.h"
#include "handle_utils.h"
#include "geometry_msgs/PointStamped.h"
#include "point_cloud_filtering/DetectHandle.h"
#include <ros/callback_queue.h>

bool detect_handle(point_cloud_filtering::DetectHandle::Request  &req,
                   point_cloud_filtering::DetectHandle::Response &res){

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

//    std::cout<< "This does repeat" << std::endl;


    ros::Subscriber sub_handle =
            nh.subscribe("cloud_in", 1, &point_cloud_filtering::HandleCropper::Callback,  &handle_cropper);

    ros::Subscriber sub_handle_centroid =
            nh.subscribe("handle_cloud", 1, &point_cloud_filtering::HandleCentroid::Callback,  &handle_centroid);

    // Until we get a good validated detection of the handle keep looping
    // Need to insert a timer here using argument from high level
    while (not handle_centroid.CheckDetection())
    {
        ros::getGlobalCallbackQueue()->callAvailable(ros::WallDuration(0.1));
    }

    res.x = handle_centroid.GetX();
    res.y = handle_centroid.GetY();
    res.z = handle_centroid.GetZ();

    return true;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "handle_grasp_pose");
  ros::NodeHandle nh;

  ros::ServiceServer service = nh.advertiseService("handle_detection", detect_handle);

  ROS_INFO("Ready to detect handles.");

  ros::spin();

  return 0;
}
