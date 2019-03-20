# orion-manipulation
Repo for packages that deal with controlling the robot's arm

## Pre-requisites
This code requires the reconstruction\_ws to be built first. 

## What does it do?
This repo enables the HSR to perform manipulation tasks such as picking up an object, opening a door, or giving the object to a human operator via respective action servers. 

The main action is pick\_up\_object\_server.py. This first uses get\_collision\_map() to call reconstruction\_ws and publish a collision map to \`known\_object\_pre\_filter\' topic. This collision map is then modified on the fly to remove an area around the object you're dealing with and then publish to 'known_object' which is listened to by the collision world. As long as collision-world is turned on, motion planning will now avoid the collision environment generated. 

The key parameter that can be tweaked is the size of the boxes. 1.5cm works well and reduction would be preferred but this has a tradeoff with demand on CPU and graphics.

The action server takes as input a tf frame as the goal for the HSR to pick up. If the tf\_frame is in the config file and is still available then a hand-made pre-grasp and grasp pose will be executed. The robot will then try to move\_to\_go. If this is not possible then it will try to move back first and then go to this position.


## Launching the servers
At WRS, these were run on the development pc and typically execute via the main tmux script in the ori\_hsr\_bringup package:

1) Launch the reconstruction
`roslaunch tmc_reconstruction reconstruction.launch camera_param_path:=~/.ros/camera_info/hsrb_gazebo.yaml vocabulary_path:=~/reconstruction_ws/DBoW2/build/small_voc.yml.gz world_frame_id_exists:=true reference_frame_id:=head_rgbd_sensor_rgb_frame rgb_image_topic:=/hsrb/head_rgbd_sensor/rgb/image_raw depth_image_topic:=/hsrb/head_rgbd_sensor/depth_registered/image_raw resolution:=0.015 is_loop_detection_enable:=false auto_start:=false output_collision_object_topic:=/known_object_pre_filter`

2) Launch the pick up server
`rosrun manipulation pick_up_object_server.py`

3) Launch the give to operator server
`rosrun manipulation give_object_to_operator_server.py`


## To do:
- [ ] Implement grasp-pose-synthesis - possibly using Andrea ten Pas code
- [ ] Investigate and fix collision mapping
- [ ] Coordinate door handle detector with vision group
- [ ] Develop door opening motion
- [ ] Fix speech client in give\_object\_to\_operator\_server.py

