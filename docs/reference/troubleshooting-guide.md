---
id: troubleshooting-guide
title: "Troubleshooting Guide"
sidebar_label: "Troubleshooting"
estimated_time: 1.5
week: 1 # Accessible throughout
module: "Reference"
prerequisites: []
learning_objectives:
  - "Provide solutions to common issues encountered during setup and development"
  - "Equip users with debugging strategies for ROS 2 and Isaac Sim"
---

# Troubleshooting Guide

This guide provides solutions to common problems you might encounter while working through the "Physical AI & Humanoid Robotics" textbook, covering setup, ROS 2 development, and Isaac Sim integration.

## General Issues

### "Command Not Found" for `ros2` or `colcon`

*   **Problem**: You try to run a `ros2` or `colcon` command and get a "command not found" error.
*   **Solution**: You likely haven't sourced your ROS 2 setup files.
    ```bash
    source /opt/ros/iron/setup.bash       # For ROS 2 Iron
    source ~/ros2_ws/install/setup.bash   # If you are in a colcon workspace
    ```
    To make this permanent, add the appropriate `source` command to your `~/.bashrc` file.

### Permissions Errors (e.g., when building packages)

*   **Problem**: You get permission denied errors, especially when trying to build or install packages.
*   **Solution**: Ensure you have correct permissions for your workspace and installation directories. Sometimes, simply running `sudo chown -R $USER:$USER ~/ros2_ws` on your workspace can fix this. Avoid using `sudo` with `colcon build` or `ros2 run` unless absolutely necessary, as it can create files with incorrect ownership.

## ROS 2 Specific Issues

### Node Not Showing Up in `ros2 node list`

*   **Problem**: You launch a ROS 2 node, but it doesn't appear when you run `ros2 node list`.
*   **Solution**:
    *   **Check for errors**: Look at the terminal where you launched the node. Are there any Python exceptions or error messages?
    *   **`rclpy.spin()`**: Ensure your Python node has `rclpy.spin(node)` or `rclpy.spin_until_future_complete()` to keep it alive and processing events.
    *   **ROS_DOMAIN_ID**: Make sure your nodes are on the same `ROS_DOMAIN_ID`. If you have multiple ROS 2 instances running, they might be on different domains. `echo $ROS_DOMAIN_ID` to check.

### Topic Echo Shows No Messages

*   **Problem**: You `ros2 topic echo /my_topic`, but nothing appears.
*   **Solution**:
    *   **Check if anyone is publishing**: `ros2 topic info /my_topic` to see if there are any publishers.
    *   **Message Type Mismatch**: Ensure the publisher and subscriber are using the exact same message type.
    *   **Frequency**: Is the publisher publishing frequently enough?
    *   **ROS_DOMAIN_ID**: As above, check `ROS_DOMAIN_ID`.

## Isaac Sim Specific Issues

### Isaac Sim Fails to Launch / Crashes

*   **Problem**: Isaac Sim does not start or crashes shortly after launching.
*   **Solution**:
    *   **NVIDIA Drivers**: Ensure you have the latest NVIDIA drivers installed and they are compatible with your Ubuntu version.
    *   **System Resources**: Isaac Sim is resource-intensive. Ensure your GPU has enough VRAM (12GB+ recommended) and you have sufficient system RAM.
    *   **Check Logs**: Look for crash logs in your Omniverse Launcher's logs directory.
    *   **Internet Connection**: Sometimes initial startup or updates require an active internet connection.

### ROS 2 Bridge Not Working

*   **Problem**: Isaac Sim is running, but ROS 2 topics/services are not appearing or responding.
*   **Solution**:
    *   **Enable Extension**: Ensure the `omni.isaac.ros2_bridge` extension is enabled in Isaac Sim (`Window -> Extensions`).
    *   **Restart Isaac Sim**: Sometimes, restarting Isaac Sim after enabling the bridge helps.
    *   **ROS_DOMAIN_ID**: Verify that the ROS 2 Bridge in Isaac Sim and your external ROS 2 environment are using the same `ROS_DOMAIN_ID`. You can usually set this in Isaac Sim's settings or a launch file.

## General Debugging Tips

*   **Check your logs**: ROS 2 nodes and Isaac Sim often provide valuable debugging information in their console output.
*   **`rqt_graph`**: Use `rqt_graph` to visualize the active nodes and topic connections. This is invaluable for understanding your ROS 2 system.
*   **`RViz2`**: Use `RViz2` to visualize sensor data, robot models, and transforms.
*   **Breakpoints and Print Statements**: Don't underestimate the power of strategically placed print statements or debugger breakpoints in your Python code.

*(Content to be expanded based on common user feedback and known issues)*
