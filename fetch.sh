#!/bin/bash

# source ros environment variables
source ~/repos/ros_noetic/install_isolated/setup.bash
# import source of required dependencies, with compatible branches
vcs import --recursive --input faster_dependencies.repos --repos src
# install binary dependencies using system's package manager(dnf/apt)
rosdep install -y -r -q --from-paths src --ignore-src --rosdistro noetic
# install pcl if above doesn't installs pcl libraries
# dnf install pcl pcl-devel # maybe only pcl-devel is req.