# TurtlesimROS2SimpleDemo

This project is a simple demonstration built using ROS 2 and the `turtlesim` package. It simulates a turtle moving to randomly generated points using heading-based control. The system is broken into multiple nodes that work together to spawn the turtle, generate target points, and control its movement based on the angle between its heading and the goal.

---

## Overview

The main goal of this demo is to explore core ROS 2 principles by creating a complete, functioning system with custom logic and multiple interacting nodes. The turtle adjusts its orientation using a smooth angle control law and adjusts its forward speed based on heading error. This results in clean, intentional motion toward random targets on the screen.

---

## Key Concepts and Skills

This project involved and reinforced the following ROS 2 concepts:

- Creating publisher and subscriber nodes
- Writing and using custom service definitions
- Managing multiple coordinated nodes
- Using `geometry_msgs/msg/Twist` for motion commands
- Implementing control logic using heading error
- Modulating linear velocity based on angular misalignment
- Building and sourcing ROS 2 workspaces
- Following best practices for modular ROS 2 projects

---

## Build and Run Instructions

The instructions below assume you're in the root of your ROS 2 workspace and the package is named `turtle_traffic`.

> Make sure to follow these steps in order. Each ROS node should be run in its **own terminal**.

### 1. Launch the Turtlesim window

Open a terminal and run:

```bash
source /opt/ros/humble/setup.bash
ros2 run turtlesim turtlesim_node
````
This will spawn a default turtle named `turtle1`, which is the turtle your nodes will control.


### 2. Build the workspace

In a separate terminal:

```bash
cd ~/your_ros2_ws  # Replace with your actual workspace path
rm -rf build/ install/ log/
colcon build --symlink-install
```

### 3. Source the environment

Once the build completes:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
```

### 4. Run the nodes (in separate terminals, in this order)

**Terminal 1** – Point generator:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run turtle_traffic point_gen
```

**Terminal 2** – Turtle manager:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run turtle_traffic turtle_move
```

**Terminal 3** – Movement controller:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run turtle_traffic perform_movement
```

You should now see the `turtle1` turtle rotating and moving to each target point in the turtlesim window.

---

## Demo

<video width="600" controls>
  <source src="media/demo.mp4" type="video/mp4">
  Your browser does not support the video tag.
</video>

---

## Author

Created by Yash Jain. If you have any questions, feel free to reach out: [yjtexas2005@gmail.com](mailto:yjtexas2005@gmail.com)

