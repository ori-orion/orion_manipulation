//
// Created by markfinean on 02/05/19.
//

#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include <tf/transform_broadcaster.h>

namespace point_cloud_filtering {

    class HandleCropper {
    public:
        HandleCropper(const ros::Publisher& cloud_pub);
        void Callback(const sensor_msgs::PointCloud2& msg);

    private:
        ros::Publisher cloud_pub_;
    };

    class HandleCentroid {
    public:
        HandleCentroid(const tf::TransformBroadcaster& br);
        void Callback(const sensor_msgs::PointCloud2& msg);

    private:
//        ros::Publisher handle_centroid_pub_;
        tf::TransformBroadcaster handle_tf_br_;
    };

}  // namespace point_cloud_filtering