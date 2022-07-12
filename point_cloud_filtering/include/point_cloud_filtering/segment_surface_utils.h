//
// Created by markfinean on 07/05/19.
//

#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include <tf/transform_broadcaster.h>
#include "pcl_conversions/pcl_conversions.h"

#include "pcl/PointIndices.h"

#include "pcl/common/angles.h"
#include "pcl/sample_consensus/method_types.h"
#include "pcl/sample_consensus/model_types.h"
#include "pcl/segmentation/sac_segmentation.h"
#include "pcl/segmentation/extract_clusters.h"

#include "pcl/filters/extract_indices.h"

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include "pcl/filters/crop_box.h"

typedef pcl::PointXYZRGB PointC;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudC;

namespace point_cloud_filtering {

    void CropCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr in_cloud,
                   pcl::PointCloud<pcl::PointXYZRGB>::Ptr out_cloud,
                   Eigen::Vector4f min_p,
                   Eigen::Vector4f max_p);

    void SegmentPlane(PointCloudC::Ptr cloud, pcl::PointIndices::Ptr indices, pcl::ModelCoefficients::Ptr coeff) ;

    void GetSurface(PointCloudC::Ptr in_cloud, PointCloudC::Ptr out_cloud, pcl::PointIndices::Ptr inliers);

    class SurfaceSegmenter {
    public:
        SurfaceSegmenter(const ros::Publisher& object_pub, const double x_in, const double y_in, const double z_in);
        void Callback(const sensor_msgs::PointCloud2& msg);

    private:
        ros::Publisher object_pub_;
        ros::Publisher placeholder_pub;
        ros::Publisher marker_pub;
        double object_x, object_y, object_z;
    };

}  // namespace point_cloud_filtering