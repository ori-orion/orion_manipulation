# orion-manipulation
Repo for packages related to manipulating objects and opening doors etc.

## Packages:
* manipulation
	-This encompasses all the actions related to interacting with objects, and manipulating with them
* point\_cloud_\filtering
	-This package is used for isolating, segmenting and clustering point clouds in order to determine grasp poses.

## Pre-requisites
* reconstruction\_ws 
	- Needs to be built first. This is required for the collision environment to be populated.
	- Creates a collision map and publishes this to the `pre\_filter' topic defined above. The collision map later gets modified on the fly to remove an area around the object you're dealing with. 
	- The key parameter that can be tweaked is the size of the boxes. 1.5cm works well and reduction would be preferred but this has a tradeoff with demand on CPU and graphics.



## Manipulation
*To fill in

To launch collision reconstruction:
```
roslaunch tmc\_reconstruction reconstruction.launch camera\_param\_path:=~/.ros/camera\_info/hsrb\_gazebo.yaml vocabulary\_path:=~/Documents/reconstruction\_ws/DBoW2/build/small\_voc.yml.gz world\_frame\_id\_exists:=true reference\_frame\_id:=head\_rgbd\_sensor\_rgb\_frame rgb\_image\_topic:=/hsrb/head\_rgbd\_sensor/rgb/image\_raw depth\_image\_topic:=/hsrb/head\_rgbd\_sensor/depth\_registered/image\_raw resolution:=0.04 is\_loop\_detection\_enable:=false auto_start:=false output\_collision\_object\_topic:=/known\_object
```
Service calls to begin, stop or reset the collision map:
```
rosservice call /tmc\_reconstruction/system/start
rosservice call /tmc\_reconstruction/system/stop
rosservice call /tmc\_reconstruction/system/reset
```

We then have an action server which takes a tf frame as a goal for the HSR to pick up. If the tf\_frame object is in the config file then a hand-made pre-grasp and grasp pose will be executed. The robot will then try to move\_to\_go. If this is not possible then it will try to move back first and then go to this position.

*rosrun the action server*

## Point\_cloud_\filtering
To launch the handle filter:
```
rosrun point\_cloud\_filtering point\_cloud\_demo cloud\_in:=/hsrb/head\_rgbd\_sensor/depth\_registered/rectified\_points
```

## Grasp pose synthesis
The following commands are used to generate grasp poses:

roslaunch openni2\_launch openni2.launch camera:=hsrb depth\_frame\_id:=/hsrb/head\_rgbd\_sensor/depth\_registered/image rgb\_frame\_id:=/hsrb/head\_rgbd\_sensor/rgb/image\_raw

roslaunch gpd tutorial1.launch cloud\_topic:=hsrb/head\_rgbd\_sensor/depth\_registered/rectified\_points


## To do:
- [x] Add manipulation actions to the orion_actions repo
- [] Modify manipulation package to use orion_actions for the actions and messages 
- [] Document `Caffe' install instructions (needed for grasp pose synthesis)
- [] Change grasp parameters for use on HSR
- [] Integrate the handle detection and grasp synthesis
- [] Implement all the actions (split this up into smaller tasks later)
- [] Refactor point cloud filtering


When were these commands needed?:
rosrun topic\_tools relay /hsrb/robot\_state/joint\_states /joint\_states
rosrun topic\_tools relay /hsrb/robot\_state/joint\_states /robot/joint\_states



