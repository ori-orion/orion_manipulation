# orion-manipulation
Repo for packages related to manipulating objects and opening doors etc.

## Packages:
* manipulation
	-This encompasses all the actions related to interacting with objects, and manipulating with them
* point\_cloud_\filtering
	-This package is used for isolating, segmenting and clustering point clouds in order to determine grasp poses.

## Pre-requisites
This code requires the reconstruction\_ws to be built first. 

*insert roslaunch command*

This creates a collision map and publishes this to the pre_filter topic defined above. The collision map later gets modified on the fly to remove an area around the object you're dealing with. 

The key parameter that can be tweaked is the size of the boxes. 1.5cm works well and reduction would be preferred but this has a tradeoff with demand on CPU and graphics.

We then have an action server which takes a tf frame as a goal for the HSR to pick up. If the tf\_frame object is in the config file then a hand-made pre-grasp and grasp pose will be executed. The robot will then try to move\_to\_go. If this is not possible then it will try to move back first and then go to this position.

*rosrun the action server*


## Manipulation
*To fill in

## Point\_cloud_\filtering
To launch the handle filter:
```
rosrun point\_cloud\_filtering point\_cloud\_demo cloud\_in:=/hsrb/head\_rgbd\_sensor/depth\_registered/rectified\_points
```
