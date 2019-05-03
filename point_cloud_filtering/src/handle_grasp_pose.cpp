#include "handle_segmentation.h"
#include "handle_utils.h"
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "geometry_msgs/PointStamped.h"

typedef const boost::function< void(const ros::Publisher &, ros::Publisher &)>  callback;


int main(int argc, char** argv) {
  ros::init(argc, argv, "handle_grasp_pose");
  ros::NodeHandle nh;

  // To publish cropped door and handle
  ros::Publisher handle_pub =
      nh.advertise<sensor_msgs::PointCloud2>("handle_cloud", 1, true);

  // To publish cropped door and handle
  ros::Publisher handle_centroid_pub =
          nh.advertise<geometry_msgs::PointStamped>("handle_centroid", 1, true);

  // Crop for the handle
  callback my_callback = boost::bind(&point_cloud_filtering::HandleCropper::Callback, &handle_cropper, &handle_centroid_pub);

  point_cloud_filtering::HandleCropper handle_cropper(handle_pub);
  ros::Subscriber sub_door =
          nh.subscribe("cloud_in", 1, my_callback);
//          nh.subscribe("cloud_in", 1, &point_cloud_filtering::HandleCropper::Callback, &handle_cropper, &handle_centroid_pub);
  ros::spin();
  return 0;
}
