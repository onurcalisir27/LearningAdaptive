#!/bin/bash

source /opt/ros/humble/setup.bash

ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/rover_control/cmd_vel_unstamped

