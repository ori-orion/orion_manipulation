//
// Created by Mark Finean on 2019-05-16.
//

#include "pcl/PointIndices.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
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

#include "geometry_msgs/PointStamped.h"
#include <eigen_conversions/eigen_msg.h>

typedef pcl::PointXYZRGB PointC;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudC;

namespace point_cloud_filtering {

    class SurfacePlacement {
    public:
        SurfacePlacement(const ros::Publisher& goal_pub);
        void Callback(const sensor_msgs::PointCloud2& msg);
        void GetHeadAngle(const sensor_msgs::JointState& msg) ;
        bool CheckDetection();
        double GetX();
        double GetY();
        double GetZ();
    private:
        ros::Publisher goal_pub_;
        double head_angle;
        bool good_detection_ ;
        double x_, y_, z_;
    };

    void SegmentSurface(PointCloudC::Ptr cloud, pcl::PointIndices::Ptr indices, Eigen::Vector3f axis);

    void RemoveSurface(PointCloudC::Ptr in_cloud, PointCloudC::Ptr out_cloud, pcl::PointIndices::Ptr inliers) ;

    void CropCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr in_cloud,
                   pcl::PointCloud<pcl::PointXYZRGB>::Ptr out_cloud,
                   Eigen::Vector4f min_p,
                   Eigen::Vector4f max_p);
}





