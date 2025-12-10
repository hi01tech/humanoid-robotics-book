---
id: digital-twin-assessment
title: "Digital Twin Assessment (Isaac Sim)"
sidebar_label: "Digital Twin Assessment"
estimated_time: 3
week: 7 # End of Digital Twin Module
module: "Digital Twin"
prerequisites:
  - "intro-isaac-sim"
  - "simulating-robots"
  - "ros2-control-isaac"
learning_objectives:
  - "Demonstrate ability to import and configure a robot in Isaac Sim"
  - "Add and configure sensors in Isaac Sim and bridge to ROS 2"
  - "Integrate `ros2_control` with a simulated robot in Isaac Sim"
  - "Control a simulated robot using ROS 2 commands"
---

# Digital Twin Assessment (Isaac Sim)

This assessment evaluates your practical skills in creating and interacting with a digital twin of a robot using NVIDIA Isaac Sim and ROS 2.

## Part 1: Theory Questions (30%)

1.  Explain the concept of Universal Scene Description (USD) and its importance in Isaac Sim.
2.  Describe the primary function of the ROS 2 Bridge in Isaac Sim. What are its main capabilities?
3.  Outline the high-level steps required to bring a URDF robot into Isaac Sim and make it controllable via `ros2_control`.
4.  What is the significance of the `controller_manager` in `ros2_control`?

## Part 2: Practical Task (70%)

You are provided with a URDF/XACRO model of a robotic arm with a simple gripper. Your task is to bring this arm into Isaac Sim, add a camera, and control it using `ros2_control`.

### Requirements:

1.  **Import URDF**: Import the provided URDF model of a 3-DOF robotic arm (plus gripper) into Isaac Sim using the URDF importer. Save it as a USD asset.
2.  **Scene Setup**: Create a new Isaac Sim scene. Add the imported robotic arm. Add a simple tabletop object and a small manipulable object (e.g., a cube) on the table.
3.  **Camera Configuration**: Attach a camera sensor to the end-effector of the robotic arm. Configure it to publish color images and camera info to ROS 2 topics.
4.  **`ros2_control` Integration**:
    *   Ensure the robotic arm is configured to be controllable by `ros2_control` within Isaac Sim.
    *   Create a `controllers.yaml` file that defines `joint_state_broadcaster` and `joint_trajectory_controller` for your robotic arm.
    *   Create a Python launch file to load the `robot_state_publisher`, the `controller_manager`, and spawn the `joint_state_broadcaster` and `joint_trajectory_controller`.
5.  **Robot Control**:
    *   From a ROS 2 terminal, send a `FollowJointTrajectory` command to the `joint_trajectory_controller` to make the robotic arm move to a pre-defined pick-and-place posture.
    *   Verify that the arm moves correctly in Isaac Sim.
6.  **Sensor Data Verification**: In a separate ROS 2 terminal, verify that the camera attached to the end-effector is publishing images to its designated ROS 2 topic.

### Evaluation Rubric

| Criteria | Needs Improvement | Meets Expectations | Exceeds Expectations |
|---|---|---|---|
| **URDF Import & Scene Setup** | Robot fails to import; scene is basic or incorrect. | Successfully imports URDF, correctly sets up a scene with arm, table, and object. | Smooth import, well-composed scene with appropriate lighting/textures, clear hierarchy. |
| **Camera Configuration** | Camera not attached or fails to publish valid ROS 2 data. | Camera attached, publishes image and info to correct ROS 2 topics. | Camera optimized (resolution, frequency), data stream verified in RViz2/image_view. |
| **`ros2_control` Integration** | `ros2_control` not functional; controllers fail to load. | Correct `controllers.yaml` and launch file; `joint_trajectory_controller` loads. | Efficient `ros2_control` setup, proper error handling, robust controller configuration. |
| **Robot Control** | Robot fails to move or moves erratically. | Successfully executes a `FollowJointTrajectory` command to a target posture. | Demonstrates understanding of trajectory points, smooth and accurate motion to target. |
| **Sensor Data Verification** | Unable to verify camera data stream. | Verifies camera image data using ROS 2 tools. | Demonstrates understanding of camera data contents and its interpretation. |
