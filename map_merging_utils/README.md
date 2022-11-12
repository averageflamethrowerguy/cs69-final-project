# auction ROS package
This ROS package corresponds to PA4. It has an auctioneer and three followers that will bid
on random target points.

## Requirements
- ROS -- tested on Melodic, but other versions may work.

## Configuration
The number of robots and their locations are determined by `robot_group.launch`. The positions to visit are determined
by a random function in `./nodes/waypoint_broadcaster`

## Build
Once cloned in a ROS workspace, e.g., `ros_workspace/src/`, run the following commands to build it:

	cd ros_workspace
    catkin_make
	
## Run
Terminal 1

    source ros_workspace/install/setup.sh
    roslaunch auction empty_world.launch

Terminal 2

	source ros_workspace/install/setup.sh
	roslaunch auction robot_group.launch

## Attribution & Licensing

Materials substantially authored by Alberto Quattrini Li. 
Copyright 2020 by Amazon.com, Inc. or its affiliates. Licensed MIT-0 - See LICENSE for further information

Source was modified by Elliot Potter in Fall 2022