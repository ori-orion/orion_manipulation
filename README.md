# orion-manipulation
Repo for packages related to manipulating objects and opening doors etc.

## Packages:
* manipulation
	-This encompasses all the actions related to interacting with objects, and manipulating with them
* point\_cloud\_filtering
	-This package is used for isolating, segmenting and clustering point clouds in order to determine grasp poses.

## Pre-requisites
* reconstruction\_ws 
	- Needs to be built first. This is required for the collision environment to be populated.
	- Creates a collision map and publishes this to the `pre\_filter' topic defined above. The collision map later gets modified on the fly to remove an area around the object you're dealing with. 
	- The key parameter that can be tweaked is the size of the boxes. 1.5cm works well and reduction would be preferred but this has a tradeoff with demand on CPU and graphics.

## Topics published
* /handle_cloud - Point cloud of the door handle (*sensor\_msgs::PointCloud2*)
* tf frame 'door_handle' - the centroid of the door handle is published as a tf frame
=======
## Pre-requisites
This code requires the reconstruction\_ws to be built first. 

## What does it do?
This repo enables the HSR to perform manipulation tasks such as picking up an object, opening a door, or giving the object to a human operator via respective action servers. 

The main action is pick\_up\_object\_server.py. This first uses get\_collision\_map() to call reconstruction\_ws and publish a collision map to \`known\_object\_pre\_filter\' topic. This collision map is then modified on the fly to remove an area around the object you're dealing with and then publish to 'known_object' which is listened to by the collision world. As long as collision-world is turned on, motion planning will now avoid the collision environment generated. 

## Opening doors
The following combination now works to test the action server. This will segment the point cloud of the handle and compute the 3D coordinates of the centre. The robot will then attempt to grasp the handle and execute a rudimentary opening motion. 
```
rosrun point_cloud_filtering handle_grasp_pose cloud_in:=/hsrb/head_rgbd_sensor/depth_registered/rectified_points
rosrun manipulation open_door_server.py
rosrun manipulation open_door_client.py 
```

## Manipulation

To launch collision reconstruction:
```
roslaunch tmc_reconstruction reconstruction.launch camera_param_path:=~/.ros/camera_info/hsrb_gazebo.yaml vocabulary_path:=~/Documents/reconstruction_ws/DBoW2/build/small_voc.yml.gz world_frame_id_exists:=true reference_frame_id:=head_rgbd_sensor_rgb_frame rgb_image_topic:=/hsrb/head_rgbd_sensor/rgb/image_raw depth_image_topic:=/hsrb/head_rgbd_sensor/depth_registered/image_raw resolution:=0.04 is_loop_detection_enable:=false auto_start:=false output_collision_object_topic:=/known_object
```
Service calls to begin, stop or reset the collision map:
```
rosservice call /tmc_reconstruction/system/start
rosservice call /tmc_reconstruction/system/stop
rosservice call /tmc_reconstruction/system/reset
```

We then have an action server which takes a tf frame as a goal for the HSR to pick up. If the tf\_frame object is in the config file then a hand-made pre-grasp and grasp pose will be executed. The robot will then try to move\_to\_go. If this is not possible then it will try to move back first and then go to this position.

*rosrun the action server*

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


The following commands are used to generate grasp poses:
```
roslaunch openni2_launch openni2.launch camera:=hsrb depth_frame_id:=/hsrb/head_rgbd_sensor/depth_registered/image rgb_frame_id:=/hsrb/head_rgbd_sensor/rgb/image_raw
```
```
roslaunch gpd tutorial1.launch cloud_topic:=hsrb/head_rgbd_sensor/depth_registered/rectified_points
roslaunch gpd tutorial1.launch cloud_topic:=**Object cloud you want to grasp**
```

## To do:
- [x] Add manipulation actions to the orion_actions repo
- [x] Hard code a door grasp pose given a point cloud of the handle
- [x] Refactor point cloud filtering
- [x] Develop door opening motions and introduce into the open door action server
- [x] Modify manipulation package to use orion_actions for the actions and messages 
- [x] Fix collision map not generating in pick up action
- [x] Test new pick up object action on AR bottle
- [x] Added service to segment an opject given the rough location of it
- [x] Create launch file to fire up the grasp synthesis
- [x] Code to segment object point clouds (crop -> remove plane ? )
- [x] Revamp pick up object server (get rid of config files, counters etc)
- PickUpObject.action
	- [x] Created server
	- [x] Tested
- OpenDoor.action
	- [x] Created server
	- [x] Tested
	- [ ] Finish opening the door fully
- PutObjectOnFloor.action
	- [x] Created server
	- [ ] Tested
- GiveObjectToOperator.action
	- [x] Created server
	- [x] Tested
- ReceiveObjectFromOperator.action
	- [x] Created server
	- [x] Tested
- PutObjectOnSurface.action
	- [ ] Created server
	- [ ] Tested
- OpenFurnitureDoor.action
	- [ ] Created server
	- [ ] Tested
- OpenDrawer.action
	- [ ] Created server
	- [ ] Tested
- CloseDrawer.action
	- [ ] Created server
	- [ ] Tested
- CloseFurnitureDoor.action
	- [ ] Created server
	- [ ] Tested
- PointToObject.action
	- [ ] Created server
	- [ ] Tested
- PlaceObjectRelative.action
	- [x] Started server (Started! Needs x,y,z in pose implementing)
	- [ ] Finish server
	- [ ] Tested
- GrabBothBinBags.action
	- [ ] Created server
	- [ ] Tested
- TurnOnBlender.action
	- [ ] Created server
	- [ ] Tested
- OpenBinLid.action
	- [ ] Created server
	- [ ] Tested
- PourInto.action
	- [ ] Created server
	- [ ] Tested
- [ ] Create launch file to launch all manipulation server nodes
- [ ] Check the axis conventention for image tf frames
- [ ] Implement a check that the handle has been grasped (or object!)!!!
- [ ] Tune grasp synthesis parameters for HSR
- [ ] Integrate the handle detection and grasp synthesis
- [ ] Document `Caffe' install instructions (needed for grasp pose synthesis)

## Notes

* The HSR stuggles to generate handles in a point cloud if their material is too specular like taht in the lab. Similarly it can't detect the glass doors.

* Grasp pose synthesis is a bit weird when applied to the door handle point cloud. It doesn't give the classic perpendicular pose as I would like. Think this should be hard coded instead.

* When were these commands needed?:
```
rosrun topic_tools relay /hsrb/robot_state/joint_states /joint_states
rosrun topic_tools relay /hsrb/robot_state/joint_states /robot/joint_states
```

The action server takes as input a tf frame as the goal for the HSR to pick up. If the tf\_frame is in the config file and is still available then a hand-made pre-grasp and grasp pose will be executed. The robot will then try to move\_to\_go. If this is not possible then it will try to move back first and then go to this position.


## Launching the servers
At WRS, these were run on the development pc and typically execute via the main tmux script in the ori\_hsr\_bringup package:

1) Launch the reconstruction
`roslaunch tmc_reconstruction reconstruction.launch camera_param_path:=~/.ros/camera_info/hsrb_gazebo.yaml vocabulary_path:=~/reconstruction_ws/DBoW2/build/small_voc.yml.gz world_frame_id_exists:=true reference_frame_id:=head_rgbd_sensor_rgb_frame rgb_image_topic:=/hsrb/head_rgbd_sensor/rgb/image_raw depth_image_topic:=/hsrb/head_rgbd_sensor/depth_registered/image_raw resolution:=0.015 is_loop_detection_enable:=false auto_start:=false output_collision_object_topic:=/known_object_pre_filter`

2) Launch the pick up server
`rosrun manipulation pick_up_object_server.py`

3) Launch the give to operator server
`rosrun manipulation give_object_to_operator_server.py`
