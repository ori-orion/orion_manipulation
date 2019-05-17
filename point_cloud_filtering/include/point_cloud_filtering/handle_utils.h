//
// Created by markfinean on 02/05/19.
//
#include "pcl_conversions/pcl_conversions.h"

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

typedef pcl::PointXYZRGB PointC;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudC;

namespace point_cloud_filtering {

    void CropCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr in_cloud,
                   pcl::PointCloud<pcl::PointXYZRGB>::Ptr out_cloud,
                   Eigen::Vector4f min_p,
                   Eigen::Vector4f max_p);

    void SegmentDoorInliers(PointCloudC::Ptr in_cloud,
                            pcl::PointIndices::Ptr points_within);

    void SegmentFurnitureInliers(PointCloudC::Ptr in_cloud,
                            pcl::PointIndices::Ptr points_within,
                            pcl::ModelCoefficients::Ptr coefficients,
                            Eigen::Vector3f axis);

//    void GetHandleFromClusters(std::vector<pcl::PointIndices> *clusters,
//                               PointCloudC::Ptr in_cloud,
//                               PointCloudC::Ptr out_cloud);
//
    void GetClusters(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
                     std::vector<pcl::PointIndices> *clusters);

    void RemoveDoor(PointCloudC::Ptr in_cloud, PointCloudC::Ptr out_cloud, pcl::PointIndices::Ptr door_inliers);

}
