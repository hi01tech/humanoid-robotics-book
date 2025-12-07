---
id: isaac-ros-slam
title: "Week 9: SLAM with Isaac ROS"
sidebar_label: "Week 9: Isaac ROS SLAM"
estimated_time: 5
week: 9
module: "NVIDIA Isaac"
prerequisites:
  - "intro-isaac-ros"
learning_objectives:
  - "Understand the principles behind Visual SLAM"
  - "Configure the Isaac ROS Visual SLAM package"
  - "Run SLAM using data from the simulated robot in Isaac Sim"
  - "Visualize the generated map and robot trajectory in RViz2"
---

# Week 9: SLAM with Isaac ROS

**Simultaneous Localization and Mapping (SLAM)** is the process by which a robot builds a map of an unknown environment while simultaneously keeping track of its own location within that map. It is a fundamental capability for autonomous navigation.

**Isaac ROS Visual SLAM** is a GPU-accelerated package that provides a real-time solution using a stereo camera. It is highly optimized for NVIDIA hardware and is designed to be robust and accurate.

## The Visual SLAM Pipeline

1.  **Input**: The node requires synchronized stereo camera images and camera calibration information.
2.  **Feature Tracking**: The algorithm identifies and tracks visual features across consecutive image frames.
3.  **Odometry Estimation**: By observing the motion of features, the algorithm estimates the robot's movement.
4.  **Loop Closure & Optimization**: The algorithm recognizes previously visited locations (loop closures) and uses this information to correct drift and build a globally consistent map.
5.  **Output**: The node outputs the robot's estimated pose (position and orientation), the map, and other visualization markers.

## Code Example: Launching Isaac ROS SLAM

Running the SLAM system involves creating a launch file that starts the `visual_slam_node` and remaps the necessary topics from your camera and robot.

```python
# slam.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Path to the Isaac ROS common launch files
    isaac_ros_common_pkg = FindPackageShare(package='isaac_ros_common').find('isaac_ros_common')

    # SLAM node
    visual_slam_node = Node(
        package='isaac_ros_visual_slam',
        executable='isaac_ros_visual_slam',
        name='visual_slam',
        parameters=[{
            'denoise_input_images': False,
            'rectified_images': True,
            'enable_slam_visualization': True,
            # Add other specific parameters here
        }],
        remappings=[
            ('stereo_camera/left/image', '/left_image_topic'),
            ('stereo_camera/left/camera_info', '/left_info_topic'),
            ('stereo_camera/right/image', '/right_image_topic'),
            ('stereo_camera/right/camera_info', '/right_info_topic')
        ]
    )
    
    # RViz2 for visualization
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', f'{isaac_ros_common_pkg}/rviz/isaac_ros_visual_slam.rviz']
    )

    return LaunchDescription([
        visual_slam_node,
        rviz_node
    ])
```

### How to Run

1.  **Prepare your robot**: In Isaac Sim, ensure you have a robot equipped with a **stereo camera** that is publishing rectified images and camera info to ROS 2 topics.
2.  **Create the launch file**: Save the launch file in your ROS 2 package. You will need to update the `remappings` to match the actual topic names published by your simulated stereo camera.
3.  **Run from inside the Isaac ROS container**:
    ```bash
    # Launch the SLAM pipeline
    ros2 launch <your_package_name> slam.launch.py
    ```
4.  **Drive the robot**: In Isaac Sim, drive your robot around the environment.
5.  **Observe in RViz2**: You will see the robot's trajectory, the generated point cloud map, and other debug information being updated in real-time.
