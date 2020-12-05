#!/bin/bash

## setup ros installation environment variables
source ~/repos/ros_noetic/install_isolated/setup.bash
## setup this repo environment variables
source ~/repos/faster/install/setup.bash
## launch simulation for navigation - gazebo, controller, planner nodes, rviz, rqt
roslaunch faster navigate_sim.launch