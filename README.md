# Assignment 1: Turtlesim UI and Distance Control

This project consists of two ROS nodes written in C++ and Python to manage the behavior of two turtles in the **Turtlesim** simulator. The aim is to dynamically control their movement based on proximity and defined boundaries.

## Node Responsibilities

### Node 1: User Interaction and Turtle Control (UI - interface.cpp)

This node spawns a second turtle (`turtle2`) in the Turtlesim environment using the `/spawn` service. It allows user input to control the velocities of either `turtle1` or `turtle2` and publishes velocity commands to the selected turtle (`/turtle1/cmd_vel` or `/turtle2/cmd_vel`). The node responds to the `/movement_allowed` topic to stop movement when restrictions are applied and briefly reverses the turtle's movement if stopped due to proximity or boundary conditions.

### Node 2: Distance and Boundary Monitoring (distance_control.py)

This node subscribes to the `/turtle1/pose` and `/turtle2/pose` topics to track the positions of both turtles in real-time. It calculates the distance between the two turtles and publishes the distance to the `/turtles_distance` topic. It also monitors boundary conditions, such as `x` or `y` coordinates falling outside the range of 1 to 10, and publishes a boolean flag to the `/movement_allowed` topic to enable or restrict movement based on proximity or boundary conditions. If the turtles are too close to each other (distance < `1.5` units) or approach the simulator boundaries, this node interrupts movement by setting `movement_allowed` to `False`. The movement direction is briefly reversed until they are within safe limits.

## Prerequisites

Ensure you have ROS installed and configured. The Turtlesim package should also be installed. Use the following command to install it:
```bash
sudo apt-get install ros-<your_ros_distro>-turtlesim
```
Set up your ROS workspace with `catkin`.

## How to Run

To start, clone this repository into your ROS workspace:
```bash
cd ~/catkin_ws/src
git clone https://github.com/grubino22/assignment1_rt
```
Build the workspace:
```bash
cd ~/catkin_ws
catkin_make
```
After building, ensure your workspace is properly sourced by adding the following line to your `.bashrc` file:
```bash
source ~/catkin_ws/devel/setup.bash
```
Then close the terminal and open a new one.

Launch the Turtlesim node with:
```bash
rosrun turtlesim turtlesim_node
```

Next, run the Python node to monitor distances:
```bash
rosrun <your_package_name> distance_control.py
```

Finally, execute the C++ node to control turtle movements:
```bash
rosrun <your_package_name> UI
```

Follow the prompts in the terminal to control `turtle1` or `turtle2`. Set velocities and observe the restrictions based on proximity or boundaries.

## Topics and Services

The `/turtles_distance` topic publishes the distance between turtles, while the `/movement_allowed` topic controls movement restrictions. The `/turtle1/pose` and `/turtle2/pose` topics provide position updates for the turtles. The `/spawn` service is used by Node 1 to spawn the second turtle (`turtle2`).

## Notes

Boundary limits are defined as `low_boundary = 1` and `high_boundary = 10`. The proximity threshold between turtles is `1.5` units. When turtles are too close to each other or to the boundaries, their movement is interrupted and briefly reversed until they reach a safe distance or position. This mechanism prevents collisions and ensures proper operation within the simulator.


