//
// Reconstructs STL meshes from Octomap occupancy map input.
//

#include <eigen_conversions/eigen_msg.h>
#include <octomap/octomap.h>
#include <octomap_msgs/GetOctomap.h>
#include <octomap_msgs/conversions.h>
#include <ros/callback_queue.h>
#include <ros/ros.h>

#include <cstdio>
#include <ctime>
#include <iostream>
using namespace std;

#include "manipulation/BoundingBox.h"
#include "manipulation/CheckPlacement.h"

class OctomapPlacementChecker {
 private:
  ros::ServiceServer check_placement_service;
  ros::ServiceClient get_octomap_client;

 public:
  explicit OctomapPlacementChecker(ros::NodeHandle* nh) {
    check_placement_service = nh->advertiseService(
        "CheckPlacement", &OctomapPlacementChecker::service_callback, this);
    get_octomap_client = nh->serviceClient<octomap_msgs::GetOctomap>("octomap_binary");
  }

  static inline octomap::point3d pointMsgToOctomap(const geometry_msgs::Point& ptMsg) {
    return octomap::point3d(ptMsg.x, ptMsg.y, ptMsg.z);
  }


  bool checkBbxClearance(octomap::OcTree* octree,
                        manipulation::BoundingBox bb){
    octomap::point3d bb_min = pointMsgToOctomap(bb.min);
    octomap::point3d bb_max = pointMsgToOctomap(bb.max);

    // Check if given space is occupied
    int num_boxes = 0;
    for (octomap::OcTree::leaf_bbx_iterator it = octree->begin_leafs_bbx(bb_min, bb_max),
                                            end = octree->end_leafs_bbx();
         it != end; ++it) {
         if (it->getOccupancy() > 0.5){
            num_boxes ++;
        }
    }
    return (num_boxes == 0);
  }

  bool checkSupported(octomap::OcTree* octree,
                    geometry_msgs::Point& _origin,
                    double maxRange){
    octomap::point3d origin = pointMsgToOctomap(_origin);
    octomap::point3d end = octomap::point3d(0, 0, 0);
    bool flag = octree->castRay(origin, octomap::point3d(0, 0, -1), end, true, maxRange);

    return flag;
  }

  bool service_callback(manipulation::CheckPlacement::Request& req,
                        manipulation::CheckPlacement::Response& res) {
    ROS_INFO("Message request received.");

    octomap_msgs::GetOctomap srv_data;

    if (!get_octomap_client.call(srv_data)) {
      ROS_ERROR("Failed to call service octomap_binary");
      return false;
    }

    octomap::AbstractOcTree* tree = octomap_msgs::binaryMsgToMap(srv_data.response.map);
    octomap::OcTree* octree = dynamic_cast<octomap::OcTree*>(tree);

    bool isAvailable = checkBbxClearance(octree, req.object_bb);
    bool isSupported = checkSupported(octree, req.center, req.maxHeight);

    res.isAvailable = isAvailable;
    res.isSupported = isSupported;

    return true;
  }
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "octomap_check_placement");
  ros::NodeHandle nh;

  OctomapPlacementChecker srv = OctomapPlacementChecker(&nh);

  ROS_INFO("%s: service ready", ros::this_node::getName().c_str());

  ros::spin();

  return 0;
}
