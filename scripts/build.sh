#!/bin/bash
source /opt/ros/humble/setup.bash

colcon build --cmake-clean-cache --symlink-install
echo "Build Succesful"

source install/setup.bash
