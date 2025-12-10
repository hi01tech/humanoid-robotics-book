---
id: ros2-quick-reference
title: "ROS 2 Quick Reference"
sidebar_label: "ROS 2 Quick Reference"
estimated_time: 1
week: 1 # Accessible throughout
module: "Reference"
prerequisites: []
learning_objectives:
  - "Provide a quick lookup for common ROS 2 commands and concepts"
---

# ROS 2 Quick Reference

This section provides a quick reference guide for common ROS 2 commands, concepts, and best practices.

## Core Commands

| Command | Description | Example |
|---|---|---|
| `ros2 run` | Run an executable from a package. | `ros2 run <package_name> <executable_name>` |
| `ros2 launch` | Run a launch file (Python, XML, YAML). | `ros2 launch <package_name> <launch_file.py>` |
| `ros2 topic list` | List active topics. | `ros2 topic list` |
| `ros2 topic info` | Display information about a topic. | `ros2 topic info /chatter` |
| `ros2 topic echo` | Display messages published on a topic. | `ros2 topic echo /chatter` |
| `ros2 topic pub` | Publish data to a topic from the command line. | `ros2 topic pub /chatter std_msgs/String "data: 'hello'"` |
| `ros2 node list` | List active nodes. | `ros2 node list` |
| `ros2 node info` | Display information about a node. | `ros2 node info /my_node` |
| `ros2 service list` | List active services. | `ros2 service list` |
| `ros2 service call` | Call a service from the command line. | `ros2 service call /add_two_ints example_interfaces/srv/AddTwoInts "{a: 1, b: 2}"` |
| `ros2 param list` | List parameters of a node. | `ros2 param list /my_node` |
| `ros2 param get` | Get a parameter's value. | `ros2 param get /my_node my_parameter` |
| `ros2 param set` | Set a parameter's value. | `ros2 param set /my_node my_parameter "new_value"` |
| `ros2 pkg list` | List installed packages. | `ros2 pkg list` |
| `ros2 interface show` | Show definition of a message, service, or action. | `ros2 interface show std_msgs/msg/String` |

## Development Workflow

1.  **Create a Workspace**:
    ```bash
    mkdir -p ~/ros2_ws/src
    cd ~/ros2_ws/
    ```
2.  **Create a Package**:
    ```bash
    cd src
    ros2 pkg create --build-type ament_python <package_name> --dependencies rclpy std_msgs
    ```
3.  **Build Workspace**:
    ```bash
    cd ~/ros2_ws
    colcon build --symlink-install
    ```
4.  **Source Setup File**:
    ```bash
    source install/setup.bash
    ```
    (Add to `~/.bashrc` for permanent sourcing)
5.  **Run a Node**:
    ```bash
    ros2 run <package_name> <executable_name>
    ```

## Important Concepts

-   **Node**: An executable process that performs computation.
-   **Topic**: A named bus for nodes to exchange messages (publish/subscribe).
-   **Message**: Data structure passed over topics.
-   **Service**: A request/reply communication method (synchronous).
-   **Action**: A goal-based, long-running communication method (asynchronous with feedback).
-   **Parameter**: Configuration values for nodes.
-   **`tf2`**: Manages coordinate frames and transforms.
-   **URDF**: XML format for robot description.
-   **Launch File**: Starts and configures multiple ROS 2 nodes.

## Environment Variables

-   `ROS_DOMAIN_ID`: Used to segment ROS 2 networks.
-   `ROS_LOG_DIR`: Directory for ROS 2 logs.

*(Content to be expanded with more advanced topics and common issues)*
