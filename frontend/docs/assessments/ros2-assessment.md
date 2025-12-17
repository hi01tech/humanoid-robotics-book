---
id: ros2-assessment
title: "ROS 2 Assessment"
sidebar_label: "ROS 2 Assessment"
estimated_time: 2
week: 5 # End of ROS 2 Module
module: "ROS 2"
prerequisites:
  - "ros2-core-concepts"
  - "ros2-launch-params"
  - "ros2-tf2"
  - "ros2-urdf"
learning_objectives:
  - "Demonstrate proficiency in ROS 2 communication patterns (publish/subscribe, services, actions)"
  - "Create and launch multiple ROS 2 nodes using Python launch files"
  - "Utilize ROS 2 parameters for node configuration"
  - "Implement coordinate frame transformations using tf2"
  - "Describe a robot's structure using URDF and visualize it"
---

# ROS 2 Assessment

This assessment evaluates your understanding and practical application of the core ROS 2 concepts covered in Weeks 4 and 5.

## Part 1: Theory Questions (30%)

Answer the following questions concisely:

1.  Explain the primary difference between a ROS 2 Service and a ROS 2 Action, and provide an example scenario for when each would be appropriate.
2.  Describe the purpose of `tf2` in ROS 2. If you have two frames, `base_link` and `camera_frame`, and you know the transform from `base_link` to `camera_frame`, how would you get the transform from `camera_frame` to `base_link`?
3.  What are the advantages of using Python launch files over manually starting nodes?
4.  Briefly explain the role of `<link>` and `<joint>` tags in a URDF file.

## Part 2: Practical Task (70%)

You are tasked with creating a simple ROS 2 system for a simulated two-wheeled robot with a pan-tilt camera.

### Requirements:

1.  **URDF Model**: Create a URDF/XACRO model of a simple mobile base with two differential wheels and a pan-tilt mechanism for a camera. Ensure all links and joints are correctly defined.
2.  **`robot_state_publisher`**: Launch your URDF model using `robot_state_publisher`.
3.  **Camera TF Broadcaster**: Create a Python ROS 2 node that continuously broadcasts a transform for a simulated camera attached to the pan-tilt mechanism. This camera should smoothly "pan" (rotate around the Z-axis) and "tilt" (rotate around the Y-axis) over time, relative to its parent link.
4.  **`tf2` Listener**: Create another Python ROS 2 node that listens to the transform from the `base_link` to the `camera_frame` and periodically prints the camera's position relative to the `base_link`.
5.  **Launch File**: Create a Python launch file that starts all the necessary components: your `robot_state_publisher`, the `camera_tf_broadcaster`, and the `tf2_listener`.
6.  **Parameters**: Your `camera_tf_broadcaster` node should have a parameter `pan_speed` (float) and `tilt_speed` (float) which can be set via the launch file to control the speed of the camera's motion.

### Evaluation Rubric

| Criteria | Needs Improvement | Meets Expectations | Exceeds Expectations |
|---|---|---|---|
| **URDF Model** | Incorrect joint/link definitions, model does not load. | Correctly defined URDF for mobile base and pan-tilt. | Well-structured XACRO with parameterized dimensions, includes visual/collision elements. |
| **TF Broadcaster** | Node crashes or transforms are incorrect/static. | Smoothly broadcasts dynamic pan/tilt transform for camera. | Robust broadcaster, handles edge cases, includes clear logging. |
| **TF Listener** | Fails to retrieve transform or prints incorrect data. | Successfully retrieves and prints camera position relative to base_link. | Efficiently queries transforms, includes error handling for transform lookup failures. |
| **Launch File** | Fails to launch components, missing nodes. | Correctly launches all required nodes with appropriate parameters. | Modular launch file, uses groups/namespaces, clear parameter handling. |
| **Parameters** | Parameters not declared or not used to control behavior. | `pan_speed` and `tilt_speed` parameters correctly control camera motion. | Parameters are declared with validation, dynamic parameter updates are handled. |
