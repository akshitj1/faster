#!/bin/bash

## source ros environment variables
source ~/repos/ros_noetic/install_isolated/setup.bash

## we use colcon build system as this is default for ros2 and future plan is to port to ros2
## usefull options:
## --packages-up-to <pkg>: build <pkg> and all its deps
## --packages-select <pkg>: build <pkg> only, skipping deps
colcon build \
--cmake-args "-Wno-dev -Wno-deprecated -DCMAKE_BUILD_TYPE=Debug" \
--packages-up-to pigeon_sim
# --packages-select faster
# --symlink-install \
## Do not abort on first package build failure, continue to rest of the packages
# --continue-on-error \

## info about rviz plugins built, used by this repo(eg. MissionModePlugin, plugin to visualize polyhedron safe space envelope)
# rospack plugins --attrib=plugin rviz