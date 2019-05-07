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

## Point\_cloud_\filtering

To launch the handle filter rosservice:
```
rosrun point_cloud_filtering handle_grasp_pose cloud_in:=/hsrb/head_rgbd_sensor/depth_registered/rectified_points
```

To make a call to handle detection:
```
rosservice call /handle_detection true
```
This will spin until a handle in front is detected and published as a tf frame. 

## Grasp pose synthesis
The following commands are used to generate grasp poses:
```
roslaunch openni2_launch openni2.launch camera:=hsrb depth_frame_id:=/hsrb/head_rgbd_sensor/depth_registered/image rgb_frame_id:=/hsrb/head_rgbd_sensor/rgb/image_raw
```
```
roslaunch gpd tutorial1.launch cloud_topic:=hsrb/head_rgbd_sensor/depth_registered/rectified_points
roslaunch gpd tutorial1.launch cloud_topic:=*Object cloud you want to grasp*
```

## To do:
- [x] Add manipulation actions to the orion_actions repo
- [x] Hard code a door grasp pose given a point cloud of the handle
- [x] Refactor point cloud filtering
- [x] Develop door opening motions and introduce into the open door action server
- [x] Modify manipulation package to use orion_actions for the actions and messages 
- [ ] Fix collision map not generating in pick up action
- [ ] Test new pick up object action on AR bottle
- [ ] Test give object server
- [ ] Create launch file to launch all manipulation server nodes
- [ ] Check the axis conventention for image tf frames
- [ ] Finish opening the door fully
- [ ] Implement a check that the handle has been grasped (or object!)!!!
- [ ] Code to segment object point clouds (crop -> remove plane ? )
- [ ] Revamp pick up object server (get rid of config files, counters etc)
- [ ] Document `Caffe' install instructions (needed for grasp pose synthesis)
- [ ] Change grasp parameters for use on HSR
- [ ] Integrate the handle detection and grasp synthesis
- [ ] Implement all the actions (split this up into smaller tasks later)

## Notes

* The HSR stuggles to generate handles in a point cloud if their material is too specular like taht in the lab. Similarly it can't detect the glass doors.

* Grasp pose synthesis is a bit weird when applied to the door handle point cloud. It doesn't give the classic perpendicular pose as I would like. Think this should be hard coded instead.

* When were these commands needed?:
```
rosrun topic_tools relay /hsrb/robot_state/joint_states /joint_states
rosrun topic_tools relay /hsrb/robot_state/joint_states /robot/joint_states
```


