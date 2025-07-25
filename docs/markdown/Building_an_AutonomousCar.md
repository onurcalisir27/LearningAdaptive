## Install ROS2 

Before we begin, we need to consider how we will communicate with our robot. But above us communicating with robot, we also need to think about how our robot will communicate with itself (sensors, compute, hardware). ROS provides you with an integrated solution, by creating an environment for your robot's components to speak freely with each other, and gives you the control to communicate with your robot, without having to be master at robotics. 

ROS is completely Free and Open Sourced, meaning anyone can install it, any time, as many times they want. There are also many pre-built projects by the community, that you can find on Github or other repository pages, which will give you a sense of how ROS works. Some very popular one's are the following:
- [Turtlebot](https://github.com/ROBOTIS-GIT/turtlebot3)
- 
There are many ROS2 distros to choose from, and all work equally great. But sometimes you have some restrictions on which distro you can use depending on the Operating System you are running. For example:

Raspberry Pi 5, only supports Ubuntu 24.04, which is best supporting ROS2 Jazzy Jalisco,
however Raspberry Pi 4 supports up to Ubuntu 22.04, which is best compatible with ROS2 Humble Hawksbill. 

For this tutorial, we will use a Raspberry Pi 4B to control our autonomous robot, so we will go with ROS2 Humble. There are rarely catastrophically different changes that happen in between distros, but it is always a good practice to make sure your code is compatible with the distribution of ROS you are working with.

This is the best page to follow to install ROS2 properly and testing it: [ROS2 Install](https://docs.ros.org/en/humble/Installation.html)

Before running code when working with ROS, always remember to source your ROS installation in your terminal. It is sufficient to run this command once when you open your terminal, but every terminal you open, you have to run it again:

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
```

Replace `$ROS_DISTRO$` with your distro, (for this tutorial "humble"), and `.bash` with the shell your system is running (".zsh", ".sh").

Alternatively, it is recommended to append the above command to your `.bashrc`, so you won't have to source your ROS installation at each time, but I only recommend doing this step if you are only using your system for ROS purposes, and running one distro at a time:

```bash
echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc
```

## Creating your first ROS package

ROS2 works in a packages system. It is ideal to create a workspace directory for your ROS related packages to go into, to not overcrowd your desktop with executable once we build our project. The most basic flow you can follow looks similar to this:

```sh
# Source your ROS installation
source /opt/ros/$ROS_DISTRO/setup.bash

# Create the working directory
mkdir -p ~/ros_ws/src
cd ~/ros_ws/src

# Create your ROS package
ros2 pkg create --build-type ament_cmake --license Apache-2.0 <your_pkg_name>

# Build your package (remember to exit to the workspace)
cd ~/ros_ws
colcon build --packages-select <your_pkg_name>

# Now if you list your directory
ls
> build install log src 
```

Now our package is where all of our code is going to live in. We will add all our Robot related files in this package, and to build/compile it, we just go to the root of your workspace, and use `colcon build`, which looks for `CMake` files to tell ROS how to build your package. 

When we are running our sourcing call to ROS, we are also referencing all ROS2 related packages that we have installed as well. This means that we can run ROS2 programs without writing any ourselves because they already live under your `/opt/ros/$ROS_DISTRO$` directory. To execute any of these programs we have two options:

```sh
# Source your ROS installation
source /opt/ros/$ROS_DISTRO/setup.bash

# The most basic ROS executable is a ROS Node, to run your node
ros2 run <ros_package_name> <ros_node_name>

# As an example lets run the ROS Visualizer
ros2 run rviz2 rviz2

# Or we could run many nodes at the same time, using launch files
ros2 launch <ros_package_name> <ros_launch_file>

# An example launch 
ros2 launch turtlesim multisim.launch.py
```

The last example launches 2 simulations of turtles in an square environment. The turtles seem stationary, but they can actually be commanded from the terminal.# ROS works in subscriptions and publishers on an object called a /topic. When you write your node, you can decide to publish on a topic, or subscribe to a topic, making your code a subscriber node, or a publisher node, or you can do both.

Open a new terminal after launching the turtle simulations and follow the steps.

```sh
# Source your ROS installation
source /opt/ros/$ROS_DISTRO/setup.bash

# To list all available topics
ros2 topic list
> /parameter_events
> /rosout
> /turtlesim1/turtle1/cmd_vel
> /turtlesim2/turtle1/cmd_vel
> ...

# There will be many topics when we write our launch files, lets check one out
ros2 topic info /turtlesim/turtle1/cmd_vel
> Type: geometry_msgs/msgs/Twist
> Publisher count: 0
> Subsriber count: 1

# So the /turtlesim/turtle1/cmd_vel topic has one subcriber node, which we assume is the turtle waiting for commands, let us give the turtle some commands
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

 The above node we run allows us to publish velocity commands and is well documented, try moving the turtle.  The turtles won't move! This is because we are publishing on the wrong topic! If you opened a new terminal, sourced ROS and ran $ ros2 topic list, you would see a new topic called: /cmd_vel. But lets fix this.
 
 ```sh
 # We can fix the wrong cmd mapping issue by adding a parameter to our node while we run it.

ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/turtlesim1/turtle1/cmd_vel

# This will run the first turtle using the keyboard key mappings on the terminal.
# You can the same way do
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/turtlesim2/turtle1/cmd_vel

# This will control the second turtle!
```

## Creating a Robot URDF

For more information, check the link below:

 [[Autonomous Car ROS2 Sim Environment#Robot Description]]

## Hardware Interface with ROS2 Control

Hardware interfaces are often the turning point of a robotics project. ROS2 is amazing for simulation work, and you can complete your algorithms and project using it, but all of the project ends if you don't have the proper tools to transition to your real life robot.

A hardware interface, simply put, is a method of giving ROS control of your robots actuators. For our project the 4 DC motors we defined in the URDF above. Without having this bridge of communication, our Robot won't be able to listen to the ROS commands we are giving to it, and execute any actions. Hardware interfaces need to be written in C++ to be compatible with ROS2, because we will use a ROS package called ROS Controls, which gives us access to a plethora of controller options, and we can choose the one we desire. Let us start by installing ROS Controls, as it rarely comes with your base ROS2 installation

```sh
sudo apt update

source /opt/ros/humble/setup.bash
sudo apt install ros-humble-ros2-control ros-humble-ros2-controllers
```

This gives us the access to use built in ROS controllers, and we can configure their parameters for our use case. To be able to use ROS controls, we need to create a ROS Hardware Interface class, in which we will give ROS access to control our hardware. But to be able to do so, we first need a method to control our hardware ourselves. There are many libraries online to choose from, granted they work with your robot setup. Or you can write one yourself.

If you are lazy and are using a differential drive robot car with quadrature encoders (the most common starter setup), then you can also use the class I wrote, by cloning it from my Github:

```sh

git clone https://github.com/onurcalisir27/DiffDrivePi.git
cd  /$(pwd)/DiffDrivePi

cp DiffDrivePi.cpp ~/ros_ws/src/my_pkg/ 
cp DiffDrivePi.hpp ~/ros_ws/src/my_pkg/include/my_pkg
```

Once we 

## Sensor Integration and Fusion



## Mapping your Environment



## Robot Localization



## Autonomous Navigation



## Improve Algorithms
