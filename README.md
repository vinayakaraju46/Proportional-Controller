# TurtleSim Motion Controller

This ROS-based C++ program controls the movement of a TurtleSim robot to navigate to a specified goal position.

## Description
This program enables a simulated turtle in the ROS TurtleSim environment to move towards a specified goal location. The motion is achieved through linear movement and angular rotation, with real-time feedback from the turtle's current position.

## Features
- Reads the turtle's current pose using a ROS subscriber.
- Moves the turtle to a specified goal position.
- Uses proportional control for smooth movement.
- Implements both linear and angular motion control.
- Uses PID-like control for steering towards the target.

## Dependencies
- ROS (Robot Operating System)
- turtlesim package

## Installation
1. Ensure ROS is installed and properly set up.
2. Clone this repository into your ROS workspace:
   ```sh
   cd ~/catkin_ws/src
   git clone <repository_url>
   ```
3. Compile the package:
   ```sh
   cd ~/catkin_ws
   catkin_make
   ```

## Usage
1. Launch the turtlesim node:
   ```sh
   rosrun turtlesim turtlesim_node
   ```
2. Run the motion control node with the desired goal coordinates:
   ```sh
   rosrun <your_package_name> robot_move <goal_x> <goal_y>
   ```
   Example:
   ```sh
   rosrun my_turtle_package robot_move 5 5
   ```

## Code Explanation
### Main Components
- **GoalPose Class**: Represents the target position with x and y coordinates.
- **getCurrentPose()**: Callback function that updates the turtle's current pose.
- **move_to_goal()**: Moves the turtle to the specified goal using proportional control.
- **rotate()**: Rotates the turtle to align with the goal position.
- **move()**: Moves the turtle forward or backward.
- **gas_pedal()**: Adjusts linear velocity.
- **steer()**: Adjusts angular velocity based on angle difference.

### Motion Logic
1. Calculate the Euclidean distance to the goal.
2. Rotate towards the goal if needed.
3. Move forward while adjusting for angular deviation.
4. Stop when the goal is reached within a specified tolerance.

## Author
Vinayaka Raju S

## License
This project is licensed under the MIT License.

