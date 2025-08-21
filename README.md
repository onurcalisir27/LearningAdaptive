# Learning Adaptive Control for Autonomous Navigation

This repository contains the `Robot Operating System 2 (ROS2)` packages that are implementing adaptive controller plugins for autonomous driving to a mobile 4 wheeled differential drive robot rover.

## Introduction
This research project implements learning-adaptive control strategies for autonomous mobile robots
in repetitive environments, supervised by Dr. Homayoon Beigi at Columbia's Nonlinear Adaptive Control
Research Lab. The custom-built differential drive AMR features a distributed
architecture with dual Raspberry Pi units—Pi4 handling hardware control through a custom ROS2 Control plugin and sensor management,
while Pi5 executes motion planning via a specialized Nav2 controller plugin that implements the learning-adaptive algorithm.

The learning controller enables the robot to improve performance through repetition, compensating for recurring errors and significantly reducing tracking errors in daily operations.
This approach is particularly valuable for AMRs in warehouses, manufacturing facilities, or delivery systems where robots follow similar routes repeatedly, allowing them to optimize their
control strategies and achieve superior trajectory tracking performance over time.

## Packages

Currently three `ROS2` packages are present under the *src/* directory,
- **rover_hardware**: this package handles the hardware level driver code for the autonomous rover, including low-level DC motor control through PWM, RGB-D camera control, and IMU processing. This package meant to be deployed on the Raspberry Pi 4 of the autonomous rover.
- **rover_control**: this package handles the autonomous control from adaptive controllers present in the research scope. This package is meant to offer new controller and planner plugins for `Nav2`, for high compatibility with the rest of the autonomous robotics community. Find more on `Nav2` under [->docs/Nav2ArchitectureAnalysis.md]. This package is prepared to be deployed on the Raspberry Pi 5 of the autonomous rover.
- **rover_sim**: this package provides a realistic simulated environment for the autonomous rover algorithms to be tested on, using the `ROS2` simulator `Gazebo`. This package provides you with a fully set up sim environment for you to test different controller algorithms using the `navigation.yaml` file.

Each package contains multiple executable launch files, prepared with accordance to `ROS2` standards using 'launch.py' format. Please find a list of executable's you can run below, and their functionality to the project.

The common way to execute a `ROS2` launch file is the following command:
```bash
$ ros2 launch <package_name> <executable.launch.py>
```

## Scripts
Before executing your code, make sure you compiled and built your environment.
Don't forget to install `ROS2` beforehand if you haven't. We provide some useful scripts under */scripts* directory. Below is example usage for a few:
```bash
$ cd scripts
$ ./ros_install.sh          # To install ROS2,if you haven't previously
$ ./ros_install_nosudo.sh   # To use Dockerfiles,if you wish (we provide one)
$ ./update.sh               # To update your system with all the dependencies
$ ./build.sh <package_name> # To build your environment (package name optional)

$ source install/setup.bash # Don't forget to source the installation after
```

After installing `ROS2` and all the package dependencies, you can go ahead with executing the following programs as you require, following the launch file template.

## Launch Files

**rover_hardware** provides you with executable's to communicate directly with the hardware components of the autonomous rover. The following launch file's are explained more in detail:

```bash

# Robot_control starts the DC motor communication pipeline and brings the system ready to listen for velocity commands to the wheels using ros2control
$ ros2 launch rover_hardware robot_control.launch.py

# Sensors starts the IMU sensor and the Sensor fusion pipeline, fusing wheel odometry information with IMU yaw rate readings to provide more reliable feedback
$ ros2 launch rover_hardware sensors.launch.py

# Camera starts the OAK-D Lite RGB-D camera with optimized parameters for the Pi 4 to handle, publishing RGB images and Depth images simultaenously
$ ros2 launch rover_hardware camera.launch.py

# Robot_launch is the main launch file you want to use, at it starts all three previously described launch files at once, with appropriate delays in between
$ ros2 launch rover_hardware robot_launch.py
```

**rover_control** provides the entry point of the research's own controller and planner plugin's in accordance with the `Nav2` environment. The following launch file's can be used to begin the autonomous planning and control:
```bash

# Start RTAB Mapping for SLAM
$ ros2 launch rover_control rtabmap.launch.py

# Start autonomous navigation with Self Tuning Regulator
$ ros2 launch rover_control str.launch.py

# Start autonomous navigation with Learning Self Tuning Regulator
$ ros2 launch rover_control lstr.launch.py

# Start autonomous navigation with Learning Adaptive Controller
$ ros2 launch rover_control lac.launch.py

# Start the particle filter for localization in the known map
$ ros2 launch rover_control amcl.launch.py
```

** Controller's are in still under production

**rover_sim** provides you with executable's to start the simulated environment with autonomous navigation. This package makes extensive use of `ROS2`'s readily available packages such as `robot_localization`, `ros2control`, `slam_toolbox`, `nav2`, `gazebo`.

```bash

# Starts RVIZ (Robot Visualization) to display the robot, useful for URDF's
$ ros2 launch rover_sim robot_display.launch.py

# Starts the Gazebo Simulation with ros2control to get the robot ready to listen for velocity commands from the controller
$ ros2 launch rover_sim simulation.launch.py

# Starts the SLAM toolbox Mapping to make a grid map of the gazebo world using a simulated LiDAR
$ ros2 launch rover_sim slam_mapping.launch.py

# Starts the RTAB mapping to make a pointcloud map of the gazebo world using a simulated RGB-D camera
$ ros2 launch rover_sim rtab.launch.py

# Starts the autonomous navigation stack with the plugins defined in the 'config/navigation.yaml' parameter file
$ ros2 launch rover_sim basic_nav.launch.py
```
