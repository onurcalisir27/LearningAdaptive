## Action Server

A common way to control long running tasks like navigation. They consist of three parts: a goal, a feedback, and a result.

Actions are built on topics and services; their functionality is similar to services except actions can be cancelled. *Actions also provide steady feedback, as opposed to services which return a single response*

Actions use a client-server model: An **action client** node sends a goal to an **action server** node that acknowledges the goal and returns a stream of *feedback and a result.
![[actionserver.png]]

Not only can the client-side stop a goal, but the server-side can as well. When the server-side chooses to stop processing a goal, it is said to "abort" the goal.

To identify all the actions in the ROS graph, we can run the command >
```bash
$ ros2 action list -t
$ ros2 action info <action_name>
```

We can also send an action goal from the cli with the following syntax >
``` bash
$ ros2 action send_goal <action_name> <action_type> <values>
```

Actions are like services that allow you to execute long running tasks, provide regular feedback, and can be canceled.

A robot system would likely use actions for navigation. An action goal could tell a robot to travel to a position. While the robot navigates to the position, it can send updates along the way (i.e. feedback), and then a final result message once it’s reached its destination.

Feedback and results can be gathered synchronously by registering callbacks with the action client. They may also be gathered by asynchronously requesting information form the shared future objects.

Action servers in the NAV2 stack are used to communicate with the highest level **Behavior Tree(BT) navigator** through a `NavigateToPose` action message. They are also used for the BT navigator to communicate with the subsequent smaller action servers to compute plans, control efforts, and recoveries.

Each would have their own unique `.action` type in `nav2_msgs` for interacting with the servers.

---
## Lifecycle Nodes and Bond

Lifecycle nodes are unique to ROS 2. They are nodes that contain state machine transitions for bringup and teardown of ROS2 servers. This helps in deterministic behavior of ROS systems in startup and shutdown. 

When a node is started, it is in **the unconfigured state,** only processing the node's constructor which should not contain any ROS networking setup or parameter reading. By the launch system or the supplied lifecycle manager, the nodes need to be transitioned to **inactive by configuring.** After, it is possible to activate the node by transitioning through the activating stage.

The configuring stage, triggering the `on_configure()` method sets up all parameters, ROS networking interfaces and for safety systems all dynamically allocated memory. 

The activation state, triggering the `on_activate()` method, will activate the ROS networking interfaces and set any states in the program to start processing information.

To shutdown, we transition **into deactivating, cleaning up, shutting down** and end in the finalized state. The networking interfaces are deactivated and stop processing, de-allocate memory, exit cleanly in those stages respectively.

---

## Behavior Trees

They are a tree structure of tasks to be completed. It creates a more scalable and human-understandable framework for defining multi-step or many state applications. 

BT provides a formal structure for navigation logic which can be both used to create complex systems but also be verifable and validated. Having the application logic centralized in the behavior tree and with independent task servers (which only communicate data over the tree) allows for formal analysis.

NAV2 uses `BehaviorTree CPP V4` as the bt library. Node plugins are created which can be constructed into a tree, inside the `BT Navigator`. The node plugins are loaded into the BT and when the XML file of the tree is parsed, the registered names are associated. At this point the tree is used to navigate. 

## Navigation Servers

Planners and controllers are at the heart of a navigation task. Recoveries are used to get the robot out of a bad situation or attempt to deal with various forms of issues to make the system fault-tolerant. Smoothers can be used for additional quality improvements of the planned path. 

Five of the action servers in NAV2 are the planner, behavior, smoother, route and controller servers. These action servers are used to host a map of algorithm plugins to complete various tasks. They also host the representation used by the algorithm plugins to compute their outputs.

**The planner, smoother, and controller servers** will be configured at runtime with the names (aliases) and types of algorithms to use. These types are the plugin-lib names that have been registered and the names are the aliases for the task. An example would be the DWB controller used with name `FollowPath` as it follows a reference path. In this case, then all parameters for DWB would be placed in that namespace, (FollowPath.<param'>)

These three servers then expose an action interface corresponding to their task. When the BT ticks the corresponding BT node, it will call the action server to process its task. The action server callback inside the server will call the chosen algorithm by its name that maps to a specific algorithm. This allows a user to abstract the algorithm used in the BT to classes of algorithms. For instance you can have N plugin controller to follow paths, avoid dynamic obstacles, or interface with a tool. Having all of these plugins in the same server allows the user to make use of a single environmental representation object, which is costly to duplicate.

For the behavior server, each of the behaviors also contains their own name, however, each plugin will also expose its own special action server. This is done because of the wide variety of behavior actions that may be created which cannot have a single simple interface to share. The behavior server also contains a costmap subscriber to the local costmap, receiving real-time updates from the controller server, to compute its tasks. We do this to avoid having multiple instances of the local costmap which are computationally expensive to duplicate.
### Planners

The task of a planner is to compute a path to complete some objective function. The path can be known as a route, depending on the nomenclature and algorithm selected. Two canonical examples are computing a plan to a goal (current position -> goal) or complete coverage (plan to cover all free space). The planner has access to a global environmental representation and sensor data buffered into it. 
Planners can be written to:
- Compute the shortest path
- Compute complete coverage path
- Compute paths along sparse or predefined routes

**The general task in Nav2 for the planner is to compute a valid and potentially optimal path from the current pose to goal pose.**

### Controllers

Controllers, also known as local planners are the way we follow the globally computed path or complete a local task. The controller will have access to a local environment representation to attempt to compute feasible control efforts for the base to follow.
Many controllers will project the robot forward in space and compute a locally feasible path at each update iteration. 
Controllers can be written to:
- Follow a path
- Dock with a charging station using detectors in the frame
- Board an elevator
- Interface with a tool

**The general task in Nav2 for a controller is to compute a valid control effort to follow the global plan.** 
### Behaviors

Recovery behaviors are a mainstay of fault-tolerant systems. **The goal of recoveries are to deal with unknown or failure conditions of the system and autonomously handle them**. 

Examples may include faults in the perception system resulting in the environmental representation being full of fake obstacles. The clear costmap recovery would then be triggered to allow the robot to move.

Another example would be if the robot was stuck due to dynamic obstacles or poor control. Backing up or spinning in place, if permissible allow the robot to move from a poor location into free space it may navigate successfully.
### Smoothers

As criteria of optimality of the path searched by a planner are usually reduced compared to reality, additional path refinement is often beneficial. Smoothers have been introduced for this purpose, typically responsible for reducing path raggedness and smoothing abrupt rotations, but also for increasing distance from obstacles and high-cost areas as the smoothers have access to global environmental representation.

Use of a separate smoother over one that is included as part of a planner is advantageous when combining different planners with different smoothers or when a specific control over smoothing is required, e.g. smoothing only a specific part of the path.

**The general task in Nav2 for a smoother is to receive a path and return its improved version.**

### Route

The route server is a specialized planner that computes a route using a navigation graph, rather than the freespace costmap. The route is computed as the optimal way from the start to the goal through the set of nodes and directional edges in the pre-defined navigation graph. This navigation graph can be generated to represent lanes, areas the robot is allowed to navigate, a teach-and-repeat route, urban roadways, and more.
#### Robot Footprints

 We set a robot’s footprint either as a circle of radius `robot_radius` or as a vector of points `footprint` representing an arbitrary polygon if the robot is non-circular. This can also be adjusted over time using the costmap’s `~/footprint` topic, which will update the polygon over time as needed due to changes in the robot’s state, such as movement of an attached manipulator, picking up a pallet, or other actions that adjust a robot’s shape. That polygon will then automatically be used by the planners and controllers.
### Waypoint Following[](https://docs.nav2.org/concepts/index.html#waypoint-following "Permalink to this heading")

Waypoint following is a basic feature of a navigation system. It tells our system how to use navigation to get to multiple destinations.

The `nav2_waypoint_follower` contains a waypoint following program with a plugin interface for specific task executors. This is useful if you need to go to a given location and complete a specific task like take a picture, pick up a box, or wait for user input. It is a nice demo application for how to use Nav2 in a sample application.

However, it could be used for more than just a sample application. There are 2 schools of thoughts for fleet managers / dispatchers:

- Dumb robot; smart centralized dispatcher
- Smart robot; dumb centralized dispatcher

In the first, the `nav2_waypoint_follower` is fully sufficient to create a production-grade on-robot solution. Since the autonomy system / dispatcher is taking into account things like the robot’s pose, battery level, current task, and more when assigning tasks, the application on the robot just needs to worry about the task at hand and not the other complexities of the system to complete the requested task. In this situation, you should think of a request to the waypoint follower as 1 unit of work (e.g. 1 pick in a warehouse, 1 security patrol loop, 1 aisle, etc) to do a task and then return to the dispatcher for the next task or request to recharge. In this school of thought, the waypoint following application is just one step above navigation and below the system autonomy application.

In the second, the `nav2_waypoint_follower` is a nice sample application / proof of concept, but you really need your waypoint following / autonomy system on the robot to carry more weight in making a robust solution. In this case, you should use the `nav2_behavior_tree` package to create a custom application-level behavior tree using navigation to complete the task. This can include sub trees like checking for the charge status mid-task for returning to dock or handling more than 1 unit of work in a more complex task.

## Setting Up Navigation Plugins

### Planner and Controller Servers

These servers may implement one or more algorithm plugins each with its own configuration tailored for a specific action or robot state. 

**The planner server is responsible for implementing algorithms that compute the robot’s path.** For example, one plugin can be configured to compute a simple shortest path between two relatively near locations while another plugin computes for a path to locations that cover the entire robot environment.

**The controller server generates the appropriate control efforts needed for a robot to complete a task in its local environment.** These tasks include but are not limited to: following a path generated by the planner server or avoiding dynamic obstacles along this path.

As mentioned before, the planner and controller servers host a map of one or multiple plugins wherein a certain plugin will be used for a certain environment, scenario, or task. For instance, the controller server can have a plugin for following a path when in long corridors to stay in the middle of the aisle, and then another plugin for avoiding dynamic obstacles in a crowded place. Selecting which planning algorithm to execute based on the robot’s task can be done through the behavior tree of your navigation system or application server.

### Selecting the Algorithm Plugins

-> **Planner Server**
The algorithm plugins for the planner server find the robot's path using a *representation of its environment captured by its different sensors*. Some of these algorithms operate by searching through the environment's grid space while others expand the robot's possible states while accounting for path feasibility.

- [NavFn](https://docs.nav2.org/configuration/packages/configuring-navfn.html) planner is a navigation planner that uses either `Dijkstra` or `A*.` 
- [Smac 2D](https://docs.nav2.org/configuration/packages/configuring-smac-planner.html) planner implements a `2D A*` algorithm using 4 or 8 connected neighborhoods with a smoother and multi-resolution query.
- [Theta Star](https://docs.nav2.org/configuration/packages/configuring-thetastar.html) planner is an implementation of $\Theta *$ using either line of sight to create non-discretely oriented path segments    

One issue to encounter when using algorithms that work on the grid space is that there is no guarantee that a drivable path can be generated for any type of robot. For example, it is not guaranteed that the `NavFn Planner` can plan a feasible path for a non-circular robot in a tight space since it uses the circular footprint of a robot (by approximating the robot’s largest cross-sectional radius) and checks for collisions per costmap grid cell. These plugins are best used on robots that can drive in any direction or rotate safely in place, such as circular differential and circular omnidirectional robots.

- [Smac Hybrid-A* planner](https://docs.nav2.org/configuration/packages/configuring-smac-planner.html) supports arbitrary shaped ackermann and legged robots. It is a highly optimized and fully reconfigurable `Hybrid-A*` implementation. This algorithm expands the robot’s candidate paths while considering the robot’s minimum turning radius constraint and the robot’s full footprint for collision avoidance. Thus, this plugin is suitable for arbitrary shaped robots that require full footprint collision checking. 

![[Pasted image 20250723163659.png]]

-> **Controller Server**
- The default controller plugin is the [DWB controller](https://docs.nav2.org/configuration/packages/configuring-dwb-controller.html). It implements a modified` Dynamic Window Approach (DWA)` algorithm with configurable plugins to compute the control commands for the robot. This controller makes use of a `Trajectory Generator plugin` that generates the set of possible trajectories. These are then evaluated by one or more `Critic plugins`, each of which may give a different score based on how they are configured. The sum of the scores from these `Critic plugins` determine the overall score of a trajectory. The best scoring trajectory then determines the output command velocity.
- [TEB controller](https://github.com/rst-tu-dortmund/teb_local_planner) is an `MPC` time optimal controller. It implements the `Timed Elastic Band (TEB)` approach which optimizes the robot’s trajectory based on its execution time, distance from obstacles, and feasibility with respect to the robot’s kinematic constraints. This controller can be used on differential, omnidirectional, ackermann, and legged robots.
- [Regulated Pure Pursuit controller (RPP)](https://docs.nav2.org/configuration/packages/configuring-regulated-pp.html) implements a variant of the pure pursuit algorithm with added regulation heuristic functions to manage collision and velocity constraints. This variation is implemented to target the needs of service or industrial robots and is suitable for use with differential, ackermann, and legged robots.
-  [Vector Pursuit Controller](https://github.com/blackcoffeerobotics/vector_pursuit_controller) implements the [Vector Pursuit algorithm](https://apps.dtic.mil/sti/pdfs/ADA468928.pdf) and calculates the command velocity using screw theory. This controller is suitable for high speed path tracking and sharp turns or when computation resources are limited. It can be used for differential, ackermann, and legged robots.

![[Pasted image 20250723163752.png]]

---
## [List of All Nav2 Plugins](https://docs.nav2.org/plugins/index.html)

---
## Writing your Own Plugins

#### New Planner Plugin

The new plugin will inherit from the base class `nav2_core::GlobalPlanner`. The base class provides 5 pure virtual methods to implement a planner plugin. The plugin will be used by the planner server to compute trajectories. Below is the list of all functions that need to be configured.

- *configure()* : Method is called at when planner server enters `on_configure` state. Ideally this method should perform declarations of ROS parameters and initialization of planner's member variables. This method takes 4 input parameters := **shared pointer to parent node, planner name, tf buffer pointer and shared pointer to costmap**
- *activate()*: Method is called when planner server enters `on_activate` state. This method should implement operations which are necessary before planner goes to an active state.
- *deactivate():* Method is called when planner server enters `on_deactivate` state. This method should implement operations which are necessary before planner goes to an inactive state.
- *cleanup()*: Method is called when planner server goes to `on_cleanup `state. This method should clean up resources which are created for the planner.
- *createPlan():* Method is called when planner server demands a global plan for specified start and goal pose. This method returns `nav_msgs::msg::Path` carrying a global plan. This method takes 3 input parameters := **start pose, goal pose, and a function to check if the action has been canceled.**

#### New Controller Plugin

Our plugin will inherit from the base class `nav2_core::Controller`. The base class provides a set of virtual methods to implement a controller plugin. These methods are called at runtime by the controller server to compute velocity commands.

- *configure()*: Method is called when controller server enters the `on_configure` state. This method should perform declarations of ROS parameters and initialization of controller's member variables. This method takes 4 input parameters:= **weak pointer to parent node, controller name, tf buffer pointer and shared pointer to costmap**
- *activate()*: Method is called when controller server enters `on_activate` state. This method should implement operations which are necessary before controllers goes to an active state
- *deactivate()*: Method is called when controller server enters `on_deactivate` state. Ideally this method should implement operations which are necessary before controller goes to an inactive state.
- *cleanup()*: Method is called when controller server goes to `on_cleanup` state. This method should clean up resources which are created for the controller.
- *setPlan()*: Method is called when the global plan is updated. Ideally this method should perform operations that transform the global plan and store it.
- *computeVelocityCommands()*: Method is called when a new velocity command is demanded by the controller server in order for the robot to follow the global path. This method returns a `geometry_msg::msg::TwistStamped` which represents the velocity command for the robot to drive. This method passes 3 parameters:= **reference to the current robot pose, its current velocity, and a pointer to the `nav2_core::GoalChecker`.**
- *cancel():* Method is called when the controller server receives a cancel request. If this method is unimplemented, the controller will immediately stop when receiving this request. If this method is implemented, the controller can perform a more graceful stop and signal the controller server when it is done.
- *setSpeedLimit()*: Method is called when it is required to limit the maximum linear speed of the robot. Speed limit could be expressed in absolute value (m/s) or in percentage from maximum robot speed. Note that typically, maximum rotational speed is being limited proportionally to the change of maximum linear speed, in order to keep current robot behavior untouched.