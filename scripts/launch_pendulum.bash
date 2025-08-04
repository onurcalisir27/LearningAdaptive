#!/bin/bash

if [ "$#" -eq 0 ]; then
	source ~/ros2/LearningAdaptive/install/setup.bash && ros2 launch rover_control pendulum.launch.py
else
	source ~/ros2/LearningAdaptive/install/setup.bash && ros2 launch rover_control pendulum.launch.py forgetting_factor:="$@"
fi
