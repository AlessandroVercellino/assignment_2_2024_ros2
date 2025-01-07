Robot Controller Node

 Overview
The `RobotController` node is a ROS 2 Python-based node that allows users to control a robot's linear and angular velocities through user input. The node publishes velocity commands to the `/cmd_vel` topic using `geometry_msgs/Twist` messages.

The robot moves with the specified velocities for 3 seconds and then stops automatically. The node also includes basic error handling for invalid inputs.

#Features
- User-friendly interface for inputting linear and angular velocities.
- Publishes velocity commands to the `/cmd_vel` topic.
- Automatically stops the robot after 3 seconds of movement.
- Logs key events and errors.

  
 Build the workspace:
   ```bash
   cd ~/ros2_ws
   colcon build
   ```
 Source the workspace:
   ```bash
   source ~/ros2_ws/install/setup.bash
   ```

Usage
To run the `RobotController` node:

```bash
ros2 run <package_name> robot_controller
```

 Running the Node
1. Launch the node:
   ros2 run <package_name> robot_controller
2. Enter the desired linear velocity (in meters per second) when prompted.
3. Enter the desired angular velocity (in radians per second) when prompted.
4. The robot will move for 3 seconds with the specified velocities and then stop.
5. Repeat the process to issue new commands.

 Node Details
 Published Topics
- **`/cmd_vel`** (`geometry_msgs/Twist`):
  Publishes the robot's linear and angular velocities.

 Error Handling
- If the user enters non-numeric input, the node logs an error and prompts the user to try again.
- Press `Ctrl+C` to safely terminate the node.





