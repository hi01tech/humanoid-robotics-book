---
id: ros2-control-isaac
title: "Controlling Robots with ros2_control"
sidebar_label: "ros2_control in Isaac Sim"
estimated_time: 5
week: 7 # Part of week 7
module: "Digital Twin"
prerequisites:
  - "simulating-robots"
learning_objectives:
  - "Understand the architecture of ros2_control"
  - "Configure a robot in Isaac Sim to be controlled by ros2_control"
  - "Use a standard ROS 2 controller to drive a simulated robot"
  - "Send velocity commands to the robot from the ROS 2 CLI"
---

# Controlling Robots with `ros2_control`

`ros2_control` is the standard framework in ROS 2 for real-time control of robots. It provides a generic interface between your high-level ROS nodes and your robot's hardware (or, in our case, simulated hardware). This allows you to write controller-agnostic code that can be reused on different robots.

## `ros2_control` in Isaac Sim

Isaac Sim has native support for `ros2_control`, which makes it an excellent platform for testing controllers before deploying them to a physical robot. By enabling the `ros2_control` bridge, Isaac Sim will automatically create the necessary interfaces to connect to your robot's simulated joints.

## The Control Loop

1.  **Isaac Sim**: Simulates the robot's joints and publishes their state (position, velocity).
2.  **`ros2_control`**: The `ros2_control` framework reads the joint states.
3.  **Controller Manager**: A `ros2_control` tool that loads and runs controllers.
4.  **Controller**: A plugin (e.g., `diff_drive_controller`) that subscribes to a command topic (like `/cmd_vel`) and calculates the required joint efforts or velocities.
5.  **`ros2_control`**: The framework sends these new commands back to the hardware interface.
6.  **Isaac Sim**: The hardware interface in Isaac Sim receives the commands and applies them to the simulated joints.

## Code Example: `ros2_control` Configuration

Configuring `ros2_control` is primarily done through YAML files, not Python scripts. You'll need to create a `controllers.yaml` file to define the controllers and their parameters.

```yaml
# my_robot_controllers.yaml
controller_manager:
  ros__parameters:
    update_rate: 100 # Hz

    diff_drive_controller:
      type: "diff_drive_controller/DiffDriveController"

    joint_state_broadcaster:
      type: "joint_state_broadcaster/JointStateBroadcaster"

diff_drive_controller:
  ros__parameters:
    left_wheel_names: ["base_to_left_wheel"]
    right_wheel_names: ["base_to_right_wheel"]
    wheel_separation: 0.4 # Corresponds to base_width
    wheel_radius: 0.05 # From our URDF
    publish_rate: 50.0

```

### How to Run

1.  **Create the YAML file**: Save the configuration above in your ROS 2 package (e.g., in a `config` directory).
2.  **Create a launch file**: Your launch file needs to load the `robot_state_publisher`, start the `controller_manager`, and then "spawn" the controllers defined in your YAML file.

    ```python
    # control.launch.py
    # This is a simplified example. A full launch file would be more complex.
    from launch import LaunchDescription
    from launch_ros.actions import Node
    
    def generate_launch_description():
        # Node to load and start controllers
        controller_manager = Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=["path/to/your/robot_description.urdf", "path/to/your/controllers.yaml"],
            output="screen",
        )

        # Node to spawn the differential drive controller
        spawn_diff_drive_controller = Node(
            package="controller_manager",
            executable="spawner",
            arguments=["diff_drive_controller"],
            output="screen",
        )
        # ... spawn other controllers like joint_state_broadcaster

        return LaunchDescription([controller_manager, spawn_diff_drive_controller])
    ```
3.  **Enable `ros2_control` in Isaac Sim**: In the properties for your robot's articulation root, enable the `ROS 2 Control` bridge.
4.  **Run the system**:
    -   Start your simulation in Isaac Sim.
    -   Run your launch file: `ros2 launch <your_package_name> control.launch.py`
    -   You can now send commands to drive the robot!
        ```bash
        ros2 topic pub /diff_drive_controller/cmd_vel_unstamped geometry_msgs/msg/Twist "{linear: {x: 0.5}, angular: {z: 0.2}}"
        ```
