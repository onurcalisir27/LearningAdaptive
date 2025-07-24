#!/bin/bash

REPO_PATH=$(find ~ -type d -name "LearningAdaptive" -print -quit)
cd "$REPO_PATH"

apt-get update && apt-get upgrade

git pull origin

if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
    sudo rosdep init
fi
rosdep update
rosdep install --from-paths src --ignore-src -r -y --rosdistro humble

source /opt/ros/humble/setup.bash
colcon build --cmake-clean-cache --packages-select rover_sim rover_control

echo "Your repository is up to date"
