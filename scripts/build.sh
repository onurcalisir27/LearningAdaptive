#!/bin/bash
source /opt/ros/humble/setup.bash

if [ "$#" -eq 0 ]; then
	colcon build --cmake-clean-cache --symlink-install
	echo "Built All Packages in Workspace"
else
	colcon build --cmake-clean-cache --symlink-install --packages-select "$@"
	echo "Built the following packages: "$@""
fi

#source $HOME/LearningAdaptive/install/setup.bash
