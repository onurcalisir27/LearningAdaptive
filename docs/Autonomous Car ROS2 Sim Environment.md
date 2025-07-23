### Overview and Motivation

This documentation is for me to log my development of an autonomous robot car, and conduct research on using Learning Adaptive Control methods as part of my [[Adaptive Control Research]] with Dr. Beigi.
### Robot Description

It is important and often useful to create a simulated environment for your robot before working on the real one; mainly to test and ship your algorithms faster and safer to not damage your real hardware with untested code.

For this we created a ROS2 package **autonomous_car** (name suspect to change), which includes our robot's `Universal Robot Description Format(URDF)` file defining its shape and dimensions in a language that ROS2 can interpret and visualize: `xacro`. Xacro is a way to ship XML language format with `macros` , which are used to create modular, shorter XML files which are expanded to larger XML files at run time. Xacro files are the bare bones of our robot, and how it connects to all the upcoming plugins and algorithms.

To create a URDF file for our robot, we need to identify key "macro's" in our robot, these can be the robot's chassis(base_link), wheels(wheel_links), sensors(see Sensors), and how they connect with each other. Using basic geometrical relations between our parts and links of our robot, we can define a chassis as a box with Length x Width x Height, and connect wheels at specific points of the chassis' coordinate frame. A basic URDF file with one chassis and one wheel could look like so:
~~~xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="autonomous_car">

<! Chassis center: Base Link -->
<link name="base_link">
	<visual>
		<origin xyz="0 0 0" rpy="0 0 0"/>
		<geometry>
			<box size="0.23 0.095 0.048"/> <!-- LxWxH: 230x95x48mm -->
		</geometry>
		<material name="blue">
			<color rgba="0.2 0.2 0.8 1.0"/>
		</material>
	</visual>
</link>
 
<!-- Front Left Wheel -->
<joint name="front_left_wheel_joint" type="continuous">
	<parent link="base_link"/>
	<child link="front_left_wheel"/>
	<origin xyz="0.04 0.0625 -0.0015" rpy="0 0 0"/>
	<axis xyz="0 1 0"/>
</joint>

<link name="front_left_wheel">
	<visual>
		<origin xyz="0 0 0" rpy="1.5708 0 0"/>
		<geometry>
			<cylinder radius="0.0325" length="0.025"/>
		</geometry>
	</visual>
</link>
~~~

Xacro files are highly customizable with their <> tag feature, and making URDF's is well documented. By completing our robot's URDF, we give it life to appear in simulations and visualizations. To spawn in our robot into the simulation, we use a builtin ROS node called: 
`robot_state_publisher` which takes in our parsed URDF file as a parameter and publishes all the transforms of our robot and their visualizations onto the `/robot_description` topic. Visualizing our robot in Rviz would look like  this:

![[RobotDisplay.png]]

Note that the URDF file's also accept collusion and physical properties to calculate inertial components of our robot, which make our simulations more realistic like we will see in the next section. For now, we will save our completed robot urdf file under the following file name and path:
- `~/autonomous_car/description/robot.urdf.xacro`

### Simulation Environment Setup

Now that we have our robot description file, and we can spawn it into simulation, we need a physics engine to create realistic testing with our robot. We need to create a world where we can test our robot with our custom algorithms, and debug our code. For this, we will use `Gazebo Harmonic`, a FOSS provided by the same team that maintain ROS : Open Robotics.

Gazebo gives us a realistic physics engine, which we can create our own custom worlds in, and test out or robot in. All parameters that the simulation is loaded in is customizable by the user, and we interact with the environment by supplying gazebo a `.sdf` file, which is very similar to XML, but for Gazebo. When we create our `sdf` file we decide which plugins to use, what objects to generate our world with, the color of the ground, the sky, the lighting... Through Gazebo plugins, we can simulate motor controllers, sensor integrations, physics forces like gravity and so much more.

For our project, we supply the following world `test.sdf` which contains a square parkour world with black lanes and a white main path, for our robot to be tested in. The world file and gazebo tagged robot urdf can be found in the following paths:
- `~/autonomous_car/worlds/test.sdf`
- `~/autonomous_car/description/robot_gazebo.urdf.xacro`

To simulate our world in Gazebo and spawn our robot in it, there 2 ROS programs that we need to launch, found in the `ros_gz_sim` package. The two programs to launch are:
- `gz_sim.launch.py`
	- `'gz_args': {~/autonomous_car/worlds/test.sdf}`
- `create`
	- arguments:
		- -topic: `/robot_description`
		- -name: `autonomous_car`
		- -x, -y, -z: xyz coordinates in the world frame we want to spawn the robot

Here is what the `test.sdf` world looks like when launched with our robots parameters

![[GazeboSim.png]]
### Hardware Interface and Control

Now that we have our robot spawned in the Gazebo simulation, we need to add means to control it. Robot control can be done in a number of ways in ROS, the easiest way would potentially be through gazebo plugins, which map velocity commands to a `joint_states_broadcaster` to rotate the wheel joints, thus moving the car in the Gazebo simulation. But for a real robot, it is not as simple to send velocity commands to the wheels. 

In a real robot, we require compute-hardware interfaces, where the main processor unit (Raspberry Pi in our case) sends motor velocity commands to the DC motors. The most common way for this is through `Pulse Width Modulation (PWM)` protocols, where we can control the average power delivered to hardware through varying the duration of on and off step pulses. Many motors work with `PWM`, and the easiest way to command these signals is through a Motor Driver Board. We will discuss more on how to control motor velocities in the real robot later, see [[ROS Hardware Interface]]. 

But with ROS and simulation, we do not require `PWM` signals or motor driver boards, what we need is hardware abstraction. This is a method to tell ROS to command hardware components, without truly knowing what hardware we are controlling. This allows us to use a generalized methodology for our robot, which can be reproducible easily across different robot platforms. To give us the hardware abstraction and a platform to build our robot control, we will use a ROS package `ROS2 Control`. This package gives us access to many robot controllers which allow us to use the hardware abstraction, we only need to write the code in their implementation to create the hardware interface between the processor and the hardware, and `ROS2 Control` acts like a middle layer between the hardware interface and ROS robot control. 

In simulation, we can use `ROS2 Control`'s Gazebo plugin tags to skip the hardware interface layer for now, and connect directly to our robot's wheel control through an existing controller configuration file. Our robot works using [Differential Drive](https://www.cs.columbia.edu/~allen/F17/NOTES/icckinematics.pdf) methods, so we can use the template from our package to define our controller for our own needs. We will create the following file:
- `~/autonomous_car/config/autonomous_car_controller.yaml`

This file will define the parameters for two `ROS2 Control` nodes that we will use to control our robot through the `controller_manager` package which will load the existing `DiffDriveController` for us, and we can edit the parameters such as wheel_separation, wheel_radius, left_wheel_names, right_wheel_names based on our robot. The other node we will load is the aforementioned `JointStateBroadcaster` which will take the commands we give our robot, and enact the changes in our simulated robot. 

So `ROS2 Control` allows us to command our robot in simulation with simple velocity commands published on the topic `/cmd_vel` and it translates the message into individual wheel velocities using the `DiffDriveController` and enacts the changes in simulation using `JointStateBroadcaster`. With this setup, if we launch our robot into the simulation, we can drive it around by using `teleop_twist_keyboard` which publishes velocity commands on the `/cmd_vel` topic. (Note that the velocity topics will need to be remapped to `/diff_drive_controller/cmd_vel` because of how the controller manager publishes the message)
```
~ » rosteleop 

This node takes keypresses from the keyboard and publishes them
as Twist/TwistStamped messages. It works best with a US keyboard layout.
---------------------------
Moving around:
   u    i    o
   j    k    l
   m    ,    .

For Holonomic mode (strafing), hold down the shift key:
---------------------------
   U    I    O
   J    K    L
   M    <    >

t : up (+z)
b : down (-z)

anything else : stop

q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%

CTRL-C to quit

currently:	speed 0.5	turn 1.0 
```
Overall, once we configure it, we can use teleoperation to control our robot in Gazebo, which gives us a base line to create our autonomous algorithms.

### Sensor Integration and Fusion

Before we can get started with autonomous behaviors for our robot, we need sensors get information from the world and our robot. There are two types of sensors our robot can use: external sensors and internal sensors. External sensors can be `cameras`, `LiDARs`, `GPS` systems and many more. These sensors allows us to see our environment, and what is ahead by giving us sensor information about the world. We use external sensors for perception and planning our trajectory.

Internal sensors in the other hand, give us valuable information about our robot in the environment. Such sensors are `odometry`(wheel encoders) and `IMU` sensors. These sensors give us feedback on the robot's state(position and orientation) with respect to its coordinate frame origin. We use internal states to estimate the instantaneous position and orientation of our robot, which is valuable information when we are autonomously driving. The internal sensors act as a closed-loop control system, giving feedback on how the robots pose has changed after a associated motor command.

To get started with using sensors, we need sensor driver nodes. These nodes allow us to read the information from our sensors as ROS topics, and we can distribute these topics to various nodes for processing and decision making. In simulation, we can use Gazebo plugins to mimic the real driver nodes and get sensor information from our simulated robot.  We will need to add sensor plugins in our `sdf` file as well as our `xacro` file to tell Gazebo to record these values while our simulation is running. Additionally, we will need to configure a ROS-Gazebo parameter bridge, to relay the topic information from Gazebo to ROS.

Example Gazebo `sdf` tags for a camera:
~~~xml
<plugin
	filename="gz-sim-sensors-system"
	name="gz::sim::systems::Sensors">
	<render_engine>ogre2</render_engine>
</plugin>
~~~

Example Gazebo `xacro` tags for a camera:
~~~xml
<gazebo reference="camera_link">
	<sensor name="camera" type="camera">
		<camera>
			<horizontal_fov>1.204</horizontal_fov>
			<image>
				<width>1280</width>
				<height>720</height>
				<format>R8G8B8</format>
			</image>
			<optical_frame_id>camera_link_optical</optical_frame_id>		
		</camera>
		<update_rate>15</update_rate>
		<topic>camera</topic>
		<gz_frame_id>camera_link</gz_frame_id>
	</sensor>
</gazebo>
~~~

And to send the sensor information from Gazebo to ROS we will use the following program:
```python
package='ros_gz_bridge',
executable='parameter_bridge',
arguments=[
# RGB image
"/camera/image@sensor_msgs/msg/Image@gz.msgs.Image",
# Camera info
"/camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo",
],
```

With this following setup, once we launch our robot into the simulation in Gazebo, we should be able to visualize our robot's camera feed under the ROS topic `/camera/image`

![[Camera Feed.png]]

We can follow a very similar workflow to configure our IMU, LiDAR or GPS. [More information on Gazebo sensors](https://gazebosim.org/docs/latest/sensors/) Note that odometry information is given to us by `ROS2 Control` where we configured previously, which gives us valuable information on our wheel revolutions, velocity and position.

The camera or LiDAR information will be used in the next section to map our environment. However there is a better way that we can use our internal sensors. See the issue with individual sensors is the process noise and uncertainty accumulates over time and can reduce the reliability of our odometry estimation greatly. This will cause our close-loop control feedback to be greatly damaged as we increasingly grow less certain that we are at a specific location in our map. This could cause issues with our controllers as our robot in reality will no longer be at the position that the controller thinks it is. To mitigate this issue, we can use an `Extended Kalman Filter` to "fuse" the odometry and IMU sensor information together to create a joint more reliable odometry publisher. 

ROS supplies us with `robot_localization` package which contains an `ekf_node` which we can use immediately by creating a config file:
- `~/autonomous_car/config/ekf.yaml`

In this config file, we will define our sensor topics, which information from these sensors we want to use, and how much we trust each sensor. Careful tuning of this file is important for reliable odometry information as our real robot will depend on its knowledge of its location in the world frame greatly during autonomous driving. After we configure this file, once we launch the `ekf_node`, we will see a new topic appear named `/odom/filtered`. This topic contains the fused sensor information and often is more reliable than regular `/odom`. While driving around the defined square in our test world, we can see the odometry information in Rviz in the form of arrows(which show the frame transformations at every update) If we complete the whole square, we see that our robot correctly keeps track of its location, which will be very helpful in the future.

![[EKF Odom.png]]
### Mapping and Localization

For full autonomous behavior, we require a reference `map` for our robot to base its understanding of the world coordinate frame which will also map the boundaries and static obstacles that the robot won't be able to traverse through and also a `cost map` which is an overlay on the standard map, but it dynamically includes temporary obstacles such as a moving obstacle, and associates costs to parts of the map based on how desirable a location on the map is for the robot to populate. In the cost map, a high cost location would be the immediate surroundings of an obstacle, and a low cost location would be empty white space. Autonomous navigation controllers would reference this cost map to calculate the lowest cost map traversal to reach goals in the map frame.
#### LiDAR SLAM
There are a few ways to create maps in ROS2. One method, widely used in industry and research, consists of using a Light Detection and Ranging `LiDAR` sensor, which can create a map based on a point cloud from the ray's the sensor emits, and maps them in terms of density. A high point cloud density area would then be classified as a `boundary`, a high cost location that the robot should likely avoid. Areas that do not reflect the ray's back to the LiDAR would then be classified as empty space, low cost. 
![[Pasted image 20250613210208.png]]
![[Lidar SLAM.png]]
`LiDAR`'s are most often used for `SLAM` (Simultaneous Localization And Mapping) algorithms, which are used to create maps of the surrounding area of the robot as the robot moves in that direction. Mapping here means constructing the map using the ray feedback, establishing boundaries and clear white space as the robot moves in its environment and gets more feedback, and it saves high probabilities spots to its memory. Localization uses a previously constructed map, to detect similar locations in the map to pinpoint place the robot in its environment, allowing us to obtain good position estimates, often much better than odometry through EKF, since we can update our beliefs as we navigate to fix drift.
#### Static Maps
Other methods, do not rely on having a LiDAR, but are a little more complicated and less popular. For example, we can start with a static map that we define, which could be good for a known repeated task that we will ask our robot to learn how to do, so we will provide the map that our robot will see and expect. This method than can use the `rgbd_camera` to perform dynamic obstacle avoidance instead of using a cost map, it will follow the ground truth supplied map as its environment, and edit its movements based on the live camera feed if there is a obstacle. We can then use regular navigation assist from ROS2, based on our ground truth manual map, and use the collision avoidance as a sort of override command method to avoid crashes. This method however has many limitations: firstly it does not provide localization capabilities, and will likely run into many issues with odometry drift on real hardware causing the robot to lose track of its real location in the map over time. Also the dynamic collision avoidance relies on the camera live feeds image relay to be perfect, which will not be the case once we are testing on the real bot.
#### RTAB Visual SLAM
A better method perhaps could be to use our camera not for collision avoidance, but rather for `Visual SLAM`. One widely used algorithm/package for Visual SLAM is called `RTAB Map` (Real Time Appearance Based Mapping). `RTAB` is well documented and can be quickly described here: 
~~~
RTAB-Map is a RGB-D, Stereo and Lidar Graph-Based SLAM approach based on an incremental appearance-based loop closure detector. The loop closure detector uses a bag-of-words approach to determinate how likely a new image comes from a previous location or a new location. When a loop closure hypothesis is accepted, a new constraint is added to the map’s graph, then a graph optimizer minimizes the errors in the map.
~~~
[Link to RTAB](https://introlab.github.io/rtabmap/)

![[RTAB Map.png]]
Along with RTAB for mapping our environment and localization, we can again use our depth camera information for dynamic obstacles and use navigation stacks to add autonomous driving. This method is closer to regular LiDAR based SLAM, as it can generalize across different environments and doesn't require a manual map, however it adds more computational complexity and localization uncertainty due to processing of the camera images.
#### April Tags
Lastly, a more unique approach to the problem can be explored. Assuming we have a map of our environment, or we create one running RTAB once only for mapping, we can then place `AprilTags` at key locations as  to easily to process `landmarks` to localize the robot in the created map environment. This is an extension of landmark based localization, which is also well documented. AprilTags are inexpensive and requires low complexity to process, and the learning adaptive controller can be thought to navigate in between AprilTags and learn the optimal paths from sequential AprilTags in a waypoint following based manner, which will allow the robot to localize itself in the map during execution of a trajectory to eliminate odometry drift, as the tags allow for full pose validation. The camera can still be used for collisions and they carry their complexity, however AprilTags allow us to localize inexpensively, saving us much in SLAM processing overheads. 

![[AprilTags.png]]

This method will require physical manipulation of the world as we place the tags, however for an repeated task environment learning project, has high merits for performance and could accelerate learning as the robot's trajectory learning can be dissected from a larger trajectory to waypoint based sub-trajectories where robot needs to learn to go in between the Tags instead.  April tags also provide millimeter-level localization accuracy compared to centimeter-level accuracy from SLAM approaches. This precision is crucial, where measuring small improvements in trajectory tracking requires highly accurate ground truth data.

### Navigation


### Learning Adaptive Controller

