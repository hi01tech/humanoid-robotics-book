---
id: urdf
title: 'Module 1: Robot Modeling with URDF'
sidebar_label: 'URDF'
---

## Modeling Robots with URDF

The Unified Robot Description Format (URDF) is an XML format used in ROS to describe all elements of a robot. This includes the robot's links, joints, sensors, and their visual appearance. A URDF file is the foundation for simulation, visualization, and collision detection.

### Core Components of a URDF File

A URDF file consists of several key tags: `<robot>`, `<link>`, and `<joint>`.

*   **`<robot>`:** The root tag of the URDF file. It has a `name` attribute.
*   **`<link>`:** This element describes a rigid part of the robot. A link has:
    *   `<visual>`: The visual appearance of the link (shape, color, texture).
    *   `<collision>`: The collision geometry of the link, used by the physics engine.
    *   `<inertial>`: The inertial properties (mass, center of mass, inertia tensor).
*   **`<joint>`:** This element describes the kinematics and dynamics of a joint, which connects two links. A joint has:
    *   `<parent>` and `<child>`: The two links connected by the joint.
    *   `<origin>`: The transform (position and orientation) of the joint relative to the parent link.
    *   `type`: The type of joint (e.g., `revolute`, `continuous`, `prismatic`, `fixed`).
    *   `<axis>` (for non-fixed joints): The axis of rotation or translation.
    *   `<limit>` (for `revolute` and `prismatic` joints): The joint limits (e.g., upper and lower angles).

### Example: A Simple Two-Link Robot Arm

Here is a basic URDF for a simple robot arm with two links and one revolute joint.

```xml
<!-- simple_arm.urdf -->
<robot name="simple_arm">

  <!-- Base Link -->
  <link name="base_link">
    <visual>
      <geometry>
        <cylinder length="0.2" radius="0.1"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.2" radius="0.1"/>
      </geometry>
    </collision>
    <inertial>
        <mass value="1"/>
        <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
    </inertial>
  </link>

  <!-- Arm Link -->
  <link name="arm_link">
    <visual>
      <geometry>
        <box size="0.5 0.1 0.1"/>
      </geometry>
      <material name="green">
        <color rgba="0 1 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.5 0.1 0.1"/>
      </geometry>
    </collision>
    <inertial>
        <mass value="0.5"/>
        <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
    </inertial>
  </link>

  <!-- Shoulder Joint -->
  <joint name="shoulder_joint" type="revolute">
    <parent link="base_link"/>
    <child link="arm_link"/>
    <origin xyz="0 0 0.1" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="10" velocity="1"/>
  </joint>

</robot>
```

### Visualizing URDF in RViz2

To visualize and control this robot model, you can use the `robot_state_publisher` and `joint_state_publisher_gui` packages.

1.  **Save the URDF:** Save the XML code above as `simple_arm.urdf`.
2.  **Create a Launch File:** A ROS 2 launch file can start all the necessary nodes.

    ```python
    # view_robot.launch.py
    import os
    from ament_index_python.packages import get_package_share_directory
    from launch import LaunchDescription
    from launch.actions import DeclareLaunchArgument
    from launch.substitutions import LaunchConfiguration
    from launch_ros.actions import Node

    def generate_launch_description():
        urdf_file_name = 'simple_arm.urdf'
        urdf = os.path.join(
            get_package_share_directory('your_package_name'),
            urdf_file_name)
        with open(urdf, 'r') as infp:
            robot_desc = infp.read()

        return LaunchDescription([
            DeclareLaunchArgument(
                'use_sim_time',
                default_value='false',
                description='Use simulation (Gazebo) clock if true'),
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                name='robot_state_publisher',
                output='screen',
                parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time'), 'robot_description': robot_desc}],
                arguments=[urdf]),
            Node(
                package='joint_state_publisher_gui',
                executable='joint_state_publisher_gui',
                name='joint_state_publisher_gui',
                output='screen'),
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                output='screen',
                arguments=['-d', os.path.join(get_package_share_directory('your_package_name'), 'urdf.rviz')])
        ])
    ```
    *Note: You'll need to create a simple RViz configuration file (`urdf.rviz`) and replace `your_package_name` with the name of your ROS 2 package.*

3.  **Run the Launch File:** When you run this launch file, RViz2 will open and display the 3D model of your robot. The `joint_state_publisher_gui` will provide a slider to move the `shoulder_joint` and see the robot model update in real-time.

URDF is a fundamental tool for robotics development in ROS 2. We will use it as the basis for simulating our humanoid robot in Isaac Sim.