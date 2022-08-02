# orion\_manipulation
Repo for packages related to manipulating objects and opening doors etc.
Currently maintained by Matt Budd (mbudd@robots.ox.ac.uk).


## Data / Process Flow
<img src="/doc/img/manipulation_data_flow.png" alt="Data Flow Diagram" width="600"/>


## Build / Installation
* See the Wiki for TMC package [installation](https://github.com/ori-orion/orion-documentation/wiki/Installing-HSR-Packages-and-Simulator-Locally) and ORIon package [initial environment setup, prerequisite installation, and building](https://github.com/ori-orion/orion-documentation/wiki/Installing-ORIon-Packages-Locally).
* As in the flow diagram above:
  * GPG and GPD are used for (optional) object grasp pose synthesis. Without grasp pose synthesis enabled, the HSR will choose a grasp which is hard coded relative to the target tf.


## orion\_manipulation Package
This repo enables the HSR to perform manipulation tasks such as picking up an object, opening a door, or giving the object to a human operator via a set of action servers. An overview of the key functionality is provided here.
This package makes use of the point\_cloud\_filtering 


### Pick up object - `pick_up_object_server.py`
`pick_up_object_server.py` makes use of point cloud input from the Xtion RGBD camera for online collision mapping. This feeds into an OctoMap server which provides occupancy estimation collision maps.
This collision map is then modified on the fly to remove an area around the object you're dealing with and then publish to 'known_object' which is listened to by the collision world. As long as collision-world is turned on, motion planning will now avoid the collision environment generated. 


#### Running / Testing
To call this action via a script: `$ rosrun manipulation pick_up_object_client.py <name_of_target_tf>`.
* An example name of a target TF would be `potted_plant`. The published tf name will be `<object_id>_01` etc. You do not need to specify the `_0n` number - manipulation will choose whichever is available.
* A node must be running to publish TFs for manipulation to attempt to pick up:
    * [Object recognition](https://github.com/ori-orion/orion-documentation/wiki/Running-the-object-detector-on-the-Alienware) will publish TFs from vision input.
    * For testing without requiring other components, [`tmc_stereo_marker_recognizer`](https://docs.hsr.io/hsr_develop_manual_en/reference/packages/tmc_stereo_marker_recognizer/PKGDOC.html?highlight=ar%20marker#ros-interface) is a built-in TMC package that will recognise and publish objects with AR tags attached, for example the HSR water™. The TF name format for these is `'ar_marker/nnn'`.
* When running this on the real robot, use Rviz to display the TF tree to confirm the name of the TF and that it is located on top of the target object.


### Open doors - `open_door_server.py`
The following combination now works to test the action server. This will segment the point cloud of the handle and compute the 3D coordinates of the centre. The robot will then attempt to grasp the handle and execute a rudimentary opening motion.

#### Running / Testing
```
rosrun point_cloud_filtering handle_grasp_pose cloud_in:=/hsrb/head_rgbd_sensor/depth_registered/rectified_points
rosrun manipulation open_door_server.py
rosrun manipulation open_door_client.py 
```


## point\_cloud\_filtering Package
This package is used for isolating, segmenting and clustering point clouds in order to determine grasp poses.


### Topics published
* /handle_cloud - Point cloud of the door handle (*sensor\_msgs::PointCloud2*)
* /object_cloud - Point cloud of the object to grasp (*sensor\_msgs::PointCloud2*)

* tf frame 'door_handle' - the centroid of the door handle is published as a tf frame
* tf frame 'drawer_handle' - the centroid of the drawer door handle is published as a tf frame
* tf frame 'goal_pose' - the centroid of the door handle is published as a tf frame


## Collision mapping

This section needs content when Octomap collision mapping is tested and working.

## Handle detection

To launch the handle filter rosservice:
```
rosrun point_cloud_filtering handle_grasp_pose cloud_in:=/hsrb/head_rgbd_sensor/depth_registered/rectified_points
```

To make a call to handle detection:
```
rosservice call /handle_detection true
```
This will spin until a handle in front is detected and published as a tf frame. 

## Object segmentation
This requires first launching the service:
```
rosrun point_cloud_filtering segment_object clo_in:=/hsrb/head_rgbd_sensor/depth_registered/rectified_points
```

To call the object segmentation:
```
rosservice call /object_segmentation **x** **y** **z**
```
where x,y,z are the coordinates (**float**) of the object in the robot's camera frame (z in front, y down, x to right)

## Grasp pose synthesis
This is now wrapped up in a launch file. 
```
roslaunch manipulation grasp_synthesis.launch
```
This will generate and publish grasps to /detect\_grasps/clustered\_grasps when point clouds are received on the topic __/object\_cloud__

Note that when using the rotation matrix \[approach, binormal, axis\] to find the quaternion matrix for hand approach, you need to take the quaternion conjugate!


## Component status:
- ReceiveObjectFromOperator.action
    - [x] `receive_object_from_operator_server.py` updated to new inheritance structure
    - [x] Tested in simulation
    - [x] Tested on real robot
- GiveObjectToOperator.action
    - [x] `receive_object_from_operator_server.py` updated to new inheritance structure
    - [x] Tested in simulation
    - [x] Tested on real robot
- PickUpObject.action
    - [x] `pick_up_object_server.py` updated to new inheritance structure
    - [x] Tested in simulation
    - [x] Tested on real robot
- PourInto.action
    - [x] `pour_into_server.py` updated to new inheritance structure
    - [x] Tested in simulation
    - [ ] Tested on real robot
- Follow.action
    - [x] `follow_server.py` updated to new inheritance structure
    - [x] Tested in simulation
    - [ ] Tested on real robot
- PointToObject.action
    - [x] `point_to_object.py` updated to new inheritance structure
    - [x] Tested in simulation
    - [ ] Tested on real robot
- PutObjectOnSurface.action
    - [x] `put_object_on_surface_server.py` updated to new inheritance structure
    - [x] Tested in simulation
    - [ ] Tested on real robot
- PutObjectOnFloor.action
    - [ ] `put_object_on_floor_server.py` updated to new inheritance structure. Should inherit from PutObjectOnSurface.
    - [ ] Tested in simulation
    - [ ] Tested on real robot
- PlaceObjectRelative.action
    - [ ] `place_object_relative_server.py` updated to new inheritance structure. Should inherit from PutObjectOnSurface.
    - [ ] Tested in simulation
    - [ ] Tested on real robot
- OpenDrawer.action
    - [ ] `open_drawer_server.py` updated to new inheritance structure. Should inherit from OpenDoor.
    - [ ] Tested in simulation
    - [ ] Tested on real robot
- CloseDrawer.action
    - [ ] `close_drawer_server.py` updated to new inheritance structure. Should inherit from OpenDrawer.
    - [ ] Tested in simulation
    - [ ] Tested on real robot
- OpenDoor.action
    - [ ] `open_door_server.py` updated to new inheritance structure
    - [ ] Tested in simulation
    - [ ] Tested on real robot
- OpenFurnitureDoor.action
    - [ ] Creation of action server `open_furniture_door_server.py` updated to new inheritance structure
    - [ ] Tested in simulation
    - [ ] Tested on real robot
- CloseFurnitureDoor.action
    - [ ] Creation of action server `close_furniture_door_server.py` updated to new inheritance structure
    - [ ] Tested in simulation
    - [ ] Tested on real robot
- OpenBinLid.action
    - [ ] `open_bin_lid_server.py` updated to new inheritance structure
    - [ ] Rewritten to be less hard-coded and reduce duplication with PickUpBinBag which also attempts to do lid things.
    - [ ] Tested in simulation
    - [ ] Tested on real robot
- PickUpBinBag.action
    - [ ] `pick_up_bin_bag_server.py` updated to new inheritance structure. Likely should inherit from PickUpObject.
    - [ ] Rewritten to be less hard-coded
    - [ ] Tested in simulation
    - [ ] Tested on real robot
- [ ] Implement a check that the handle has been grasped (or object!)!!!
- [ ] Cleanup of unused actions e.g. GrabBothBinBags.action, TurnOnBlender.action
- [ ] Manage all parameters (defaults and specific to specific action servers) through rosparam.
    - [ ] Add YAML files to hold default parameters.
- [ ] Update messages so bounding box sizes are passed through message interface rather than hard-coded on C++ node side.
- [ ] Additional visualisation:
    - [ ] Add a flag to octomap_to_reconstruction_service to publish octomap boxes as rviz markers if stl files not available over network
    - [ ] Publish wireframe boxes for bounding boxes from other C++ nodes, in the same way as `octomap_to_reconstruction_service`
## Notes

* The HSR struggles to generate handles in a point cloud if their material is too specular like that in the lab. Similarly it can't detect the glass doors.
* Grasp pose synthesis is a bit weird when applied to the door handle point cloud. It doesn't give the classic perpendicular pose as I would like. Think this should be hard coded instead.
* The omni_base commands fail if the pose integrator is killed (as was required for using AMCL move\_base)
