---
id: intro-isaac-ros
title: "Week 8: Introduction to Isaac ROS"
sidebar_label: "Week 8: Intro to Isaac ROS"
estimated_time: 4
week: 8
module: "NVIDIA Isaac"
prerequisites:
  - "simulating-robots"
learning_objectives:
  - "Understand what Isaac ROS provides for robotics development"
  - "Identify key GPU-accelerated perception packages"
  - "Set up the recommended Docker-based development environment for Isaac ROS"
  - "Run a basic Isaac ROS pipeline to verify the installation"
---

# Week 8: Introduction to Isaac ROS

**Isaac ROS** is a collection of hardware-accelerated packages for the Robot Operating System (ROS) 2. These packages are optimized to run on NVIDIA's Jetson platform and GPUs, providing a significant performance boost for common robotics tasks like perception, navigation, and manipulation.

By leveraging the power of the GPU, Isaac ROS allows for more complex algorithms to run in real-time on resource-constrained systems.

## Key Package Categories

Isaac ROS provides solutions for:
-   **Localization & Mapping**: GPU-accelerated SLAM and odometry.
-   **Object Detection & Tracking**: Optimized nodes for popular models like YOLO, and object trackers.
-   **Image Processing**: Common computer vision operations (rectification, resizing, etc.) that run on the GPU, freeing up the CPU.
-   **AprilTag Detection**: High-performance detection of fiducial markers.

## Development Environment

The officially supported way to use Isaac ROS is through a Docker-based development environment. NVIDIA provides pre-built Docker images that contain ROS 2, all the Isaac ROS packages, and the necessary libraries and dependencies. This ensures a consistent and reproducible development environment.

## Code Example: Isaac ROS Docker Setup

You will typically start by cloning the `isaac_ros_common` repository, which includes scripts to help you manage the development environment.

```bash
# Clone the common utilities for Isaac ROS
git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common.git

cd isaac_ros_common

# Run the script to pull the Docker image and start the container
# This will mount your current directory into the container
./scripts/run_dev.sh
```

### Inside the Container

Once inside the Docker container, you have a fully configured ROS 2 and Isaac ROS workspace.

```bash
# Example of running a simple Isaac ROS pipeline (image resizing)

# In one terminal, start a camera feed (can be a real camera or a simulated one)
ros2 run camera_ros camera_node

# In another terminal inside the container, run the resizing node
ros2 run isaac_ros_image_proc isaac_ros_resize --ros-args -p output_width:=320 -p output_height:=240

# You can now see the original /image_raw topic and the new /resize/image_raw topic
ros2 topic list
```

This simple example shows the core concept: an Isaac ROS node (`isaac_ros_resize`) subscribes to a topic, performs a GPU-accelerated operation, and publishes the result on a new topic. More complex pipelines are built by chaining these nodes together.
