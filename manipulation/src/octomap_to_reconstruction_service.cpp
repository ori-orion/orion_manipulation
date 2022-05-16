//
// Created by Mark Finean on 2019-07-04.
//
#include "manipulation/GetReconstruction.h"
//#include "handle_utils.h"

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

bool octomap_to_reconstruction(manipulation::GetReconstruction::Request &req,
                               manipulation::GetReconstruction::Response &res){

    ROS_INFO("Message request received.");
    ros::NodeHandle nh;

    ros::ServiceClient client = nh.serviceClient<octomap_msgs::GetOctomap>("octomap_binary");
    octomap_msgs::GetOctomap srv;
    if (client.call(srv))
    {
      res.resp.header = srv.response.map.header;

      octomap::AbstractOcTree* tree = octomap_msgs::binaryMsgToMap(srv.response.map);
      octomap::OcTree* octree = dynamic_cast<octomap::OcTree*>(tree);
      
      for(octomap::OcTree::leaf_iterator it = octree->begin_leafs(),
       end=octree->end_leafs(); it!= end; ++it) {

        if (it.getOccupancy() > 0.5) {
          tmc_geometry_msgs::OrientedBoundingBox box;
          box.center.x = it.getCoordinate().x();
          box.center.y = it.getCoordinate().y();
          box.center.z = it.getCoordinate().z();
          box.extents.x = it.getSize();
          box.extents.y = it.getSize();
          box.extents.z = it.getSize();
          box.axis.x = 1.0;
          box.axis.y = 0.0;
          box.axis.z = 0.0;
          box.angle = 0.0;
          res.resp.boxes.push_back(box);
        }
      } 
    }
    else
    {
      ROS_ERROR("Failed to call service octomap_binary");
      return 1;
    }

    return 1;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "octomap_to_reconstruction");
    ros::NodeHandle nh;

    ros::ServiceServer service = nh.advertiseService("GetReconstruction", octomap_to_reconstruction);

    ROS_INFO("Ready to convert octomap service output to tmc_reconstruction equivalent");

    ros::spin();

    return 0;
}
