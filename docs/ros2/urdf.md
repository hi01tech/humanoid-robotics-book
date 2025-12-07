---
id: ros2-urdf
title: "Robot Modeling with URDF"
sidebar_label: "URDF Robot Models"
estimated_time: 4
week: 5 # Part of week 5
module: "ROS 2"
prerequisites:
  - "ros2-tf2"
learning_objectives:
  - "Understand the URDF (Unified Robot Description Format) for describing a robot's structure"
  - "Create a simple URDF file for a two-wheeled robot"
  - "Use XACRO to simplify URDF files"
  - "Visualize a robot model in RViz2 using robot_state_publisher"
---

# Robot Modeling with URDF

A **URDF (Unified Robot Description Format)** file is an XML file used in ROS to describe all the physical elements of a robot model. This description includes the robot's links (its physical parts), joints (which connect the links), and their visual and collision properties.

## XACRO: Better URDFs

Writing URDFs by hand can be repetitive. **XACRO (XML Macros)** is a tool that lets you create more readable and maintainable URDF files. You can define constants, create simple mathematical expressions, and use macros to avoid copying and pasting. XACRO files are processed to generate a final URDF file.

## Key Components of URDF

-   **`<link>`**: Describes a rigid part of the robot. It has elements for its visual appearance (`<visual>`), collision geometry (`<collision>`), and inertial properties (`<inertial>`).
-   **`<joint>`**: Describes the connection between two links. It specifies the parent and child links, the joint type (e.g., `revolute`, `continuous`, `prismatic`, `fixed`), and its axis of motion.

## Code Examples

### 1. Simple Robot XACRO File

This example defines a simple two-wheeled robot with a caster wheel.

```xml
<!-- two_wheel_robot.urdf.xacro -->
<robot name="two_wheel_robot" xmlns:xacro="http://www.ros.org/wiki/xacro">

  <xacro:property name="wheel_radius" value="0.05" />
  <xacro:property name="wheel_length" value="0.02" />
  <xacro:property name="base_width" value="0.2" />

  <link name="base_link">
    <visual>
      <geometry>
        <box size="${base_width} 0.3 0.1"/>
      </geometry>
    </visual>
  </link>

  <link name="left_wheel_link">
    <visual>
      <geometry>
        <cylinder radius="${wheel_radius}" length="${wheel_length}"/>
      </geometry>
    </visual>
  </link>

  <joint name="base_to_left_wheel" type="continuous">
    <parent link="base_link"/>
    <child link="left_wheel_link"/>
    <origin xyz="0 ${base_width/2 + wheel_length/2} 0" rpy="-1.5707 0 0"/>
    <axis xyz="0 0 1"/>
  </joint>

  <link name="right_wheel_link">
    <visual>
      <geometry>
        <cylinder radius="${wheel_radius}" length="${wheel_length}"/>
      </geometry>
    </visual>
  </link>

  <joint name="base_to_right_wheel" type="continuous">
    <parent link="base_link"/>
    <child link="right_wheel_link"/>
    <origin xyz="0 -${base_width/2 + wheel_length/2} 0" rpy="1.5707 0 0"/>
    <axis xyz="0 0 -1"/>
  </joint>

</robot>
```

### 2. Launch File for Visualization

This launch file converts the XACRO to URDF, starts the `robot_state_publisher` to broadcast the robot's transforms, and opens RViz2 for visualization.

```python
# display_robot.launch.py
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    pkg_path = os.path.join(get_package_share_directory('<your_package_name>'))
    xacro_file = os.path.join(pkg_path, 'urdf', 'two_wheel_robot.urdf.xacro')
    robot_description_config = xacro.process_file(xacro_file)
    robot_description = robot_description_config.toxml()

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'robot_description': robot_description}]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(pkg_path, 'config', 'view_robot.rviz')]
    )

    return LaunchDescription([
        robot_state_publisher_node,
        rviz_node
    ])
```

### How to Run

1.  Place the XACRO file in a `urdf` directory in your package.
2.  Create a simple RViz2 configuration file (`view_robot.rviz`) and save it in a `config` directory.
3.  Create the launch file in your `launch` directory.
4.  Build and launch: `ros2 launch <your_package_name> display_robot.launch.py`.
5.  In RViz2, set the "Fixed Frame" to `base_link` and add the "RobotModel" display to see your robot.
