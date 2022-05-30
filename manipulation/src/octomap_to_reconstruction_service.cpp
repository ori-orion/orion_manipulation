"""
Reconstructs STL meshes from Octomap occupancy map input.
"""

#include "manipulation/GetReconstruction.h"
#include "manipulation/BoundingBox.h"
//#include "handle_utils.h"

#include "../include/stl-creator/vec3.h"
#include "../include/stl-creator/triangle.h"
#include "../include/stl-creator/mesh.h"

#include <ros/ros.h>
#include <ros/callback_queue.h>

#include <octomap/octomap.h>
#include <octomap_msgs/conversions.h>
#include <octomap_msgs/GetOctomap.h>
#include <std_msgs/Empty.h>
#include <tmc_mapping_msgs/CollisionMap.h>

#include <ctime>
#include <iostream>
#include <cstdio>

static inline octomap::point3d pointMsgToOctomap(const geometry_msgs::Point& ptMsg){
    return octomap::point3d(ptMsg.x, ptMsg.y, ptMsg.z);
    }

bool octomap_to_reconstruction(manipulation::GetReconstruction::Request &req,
                               manipulation::GetReconstruction::Response &res){

    ROS_INFO("Message request received.");
    ros::NodeHandle nh;

    res.flag = false;

    ros::ServiceClient client = nh.serviceClient<octomap_msgs::GetOctomap>("octomap_binary");
    octomap_msgs::GetOctomap srv;
    if (client.call(srv))
    {
        octomap::AbstractOcTree* tree = octomap_msgs::binaryMsgToMap(srv.response.map);
        octomap::OcTree* octree = dynamic_cast<octomap::OcTree*>(tree);
        double thresMin = octree->getClampingThresMin();

        // iterating through areas to prune
        std::vector<manipulation::BoundingBox> crop_bbs = req.crop_bbs;
        for (auto crop_bb: crop_bbs) {
            // extracting the boundary boxes for the area of interest
            octomap::point3d crop_min = pointMsgToOctomap(crop_bb.min);
            octomap::point3d crop_max = pointMsgToOctomap(crop_bb.max);
            // iterate through all boxes inside te pruning region and prune them
            for (octomap::OcTree::leaf_bbx_iterator it = octree->begin_leafs_bbx(crop_min,crop_max),
            end=octree->end_leafs_bbx(); it!= end; ++it) {
                it->setLogOdds(octomap::logodds(thresMin));
                //m_octree->updateNode(it.getKey(), -6.0f);
            }
        }
        octree->updateInnerOccupancy();

        // extracting the boundary boxes for the area of interest
        octomap::point3d ex_min = pointMsgToOctomap(req.external_bb.min);
        octomap::point3d ex_max = pointMsgToOctomap(req.external_bb.max);

        Mesh all_mesh, box_mesh;
        int num_boxes = 0;

        // iterate through all existing boxes in the area of interest
        for (octomap::OcTree::leaf_bbx_iterator it = octree->begin_leafs_bbx(ex_min, ex_max),
            end=octree->end_leafs_bbx(); it!= end; ++it) {

            if (it->getOccupancy() > 0.5) {
              // add box
                box_mesh = create_cube();
                box_mesh.scale(Vec3(it.getSize(), it.getSize(), it.getSize()));
                box_mesh.translate(Vec3(it.getCoordinate().x(), it.getCoordinate().y(), it.getCoordinate().z()));

                all_mesh += box_mesh;
                num_boxes++;
            }
        }
        //save mesh to file
        if (num_boxes > 0)
        {
            all_mesh.stl_write_binary(req.stl_path);
            res.flag = true;
        }
        else
        {
            res.flag = false;
        }
    }
    else
    {
        ROS_ERROR("Failed to call service octomap_binary");
        return false;
    }

    return true;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "octomap_to_reconstruction");
    ros::NodeHandle nh;

    ros::ServiceServer service = nh.advertiseService("GetReconstruction", octomap_to_reconstruction);

    ROS_INFO("Ready to convert octomap service output to tmc_reconstruction equivalent");

    ros::spin();

    return 0;
}
