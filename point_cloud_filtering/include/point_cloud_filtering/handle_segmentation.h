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
        HandleCropper(const ros::Publisher& cloud_pub, const ros::Publisher& door_pub);
        void Callback(const sensor_msgs::PointCloud2& msg);

    private:
        ros::Publisher cloud_pub_;
        ros::Publisher door_pub_;
    };

    class HandleCentroid {
    public:
        HandleCentroid(const tf::TransformBroadcaster& br);
        void Callback(const sensor_msgs::PointCloud2& msg);
        bool CheckDetection();
        double GetX();
        double GetY();
        double GetZ();
    private:
//        ros::Publisher handle_centroid_pub_;
        tf::TransformBroadcaster handle_tf_br_;
        bool good_detection_ ;
        double x_, y_, z_;
    };

}  // namespace point_cloud_filtering