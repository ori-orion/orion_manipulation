# orion-manipulation
Repo for packages that deal with controlling the robot's arm

## Pre-requisite
This code requires the reconstruction_ws to be built first. 

*insert roslaunch command*

This creates a collision map and publishes this to the pre_filter topic defined above. The collision map later gets modified on the fly to remove an area around the object you're dealing with. 

The key parameter that can be tweaked is the size of the boxes. 1.5cm works well and reduction would be preferred but this has a tradeoff with demand on CPU and graphics.

We then have an action server which takes a tf frame as a goal for the HSR to pick up. If the tf_frame object is in the config file then a hand-made pre-grasp and grasp pose will be executed. The robot will then try to move_to_go. If this is not possible then it will try to move back first and then go to this position.

*rosrun the action server*
