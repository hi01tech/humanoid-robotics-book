---
id: isaac-ros-detection
title: "Week 10: Object Detection with Isaac ROS"
sidebar_label: "Week 10: Object Detection"
estimated_time: 5
week: 10
module: "NVIDIA Isaac"
prerequisites:
  - "intro-isaac-ros"
learning_objectives:
  - "Understand the pipeline for deep learning inference in ROS 2"
  - "Use the Isaac ROS package for object detection (e.g., with YOLOv8)"
  - "Connect a simulated camera to the detection node"
  - "Visualize the detected object bounding boxes in RViz2"
---

# Week 10: Object Detection with Isaac ROS

Object detection is a core perception task in robotics, allowing a robot to identify and locate objects in its environment. **Isaac ROS** provides optimized packages that leverage NVIDIA's TensorRT to perform high-performance deep learning inference on camera streams.

## The Inference Pipeline

A typical deep learning pipeline in Isaac ROS involves several nodes working together:
1.  **Camera Node**: Publishes raw image data (from a real or simulated camera).
2.  **Image Format Converter Node**: Converts the image into a format suitable for the neural network.
3.  **TensorRT Node**: Performs the actual inference using a model that has been optimized with TensorRT.
4.  **Parser Node**: Interprets the raw output from the neural network and converts it into standard ROS 2 messages, such as `vision_msgs/Detection2DArray`.
5.  **Visualization Node**: A helper node to draw the bounding boxes onto an image for easy visualization.

## Code Example: Launching Object Detection

This launch file creates a pipeline for running YOLOv8-based object detection on a camera feed.

```python
# object_detection.launch.py
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    
    # The image source can be a simulated camera or a real one
    image_source_topic = '/camera/image_raw'

    # YOLOv8 model has been converted to an engine file for TensorRT
    model_path = '/path/to/your/yolov8.engine'

    container = ComposableNodeContainer(
        name='detection_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            # 1. Inference Node
            ComposableNode(
                package='isaac_ros_tensor_rt',
                plugin='nvidia::isaac_ros::dnn_inference::TensorRTNode',
                name='tensor_rt_node',
                parameters=[{'model_file_path': model_path}],
                remappings=[('tensor_pub', 'tensor_pub'), 
                            ('tensor_sub', 'tensor_sub')]
            ),
            # 2. YOLOv8 Decoder Node
            ComposableNode(
                package='isaac_ros_yolov8',
                plugin='nvidia::isaac_ros::yolov8::YoloV8DecoderNode',
                name='yolov8_decoder_node',
            ),
            # 3. Image Preprocessor
            ComposableNode(
                package='isaac_ros_image_proc',
                plugin='nvidia::isaac_ros::image_proc::ResizeNode',
                name='resize_node',
                parameters=[{'output_width': 640, 'output_height': 640}],
                remappings=[('image_in', image_source_topic),
                            ('image_out', 'resized_image')]
            )
        ],
        output='screen'
    )
    
    return LaunchDescription([container])
```

### How to Run

1.  **Get a Model**: You need a YOLOv8 model that has been converted to a TensorRT `.engine` file. The Isaac ROS documentation provides instructions on how to do this.
2.  **Create the launch file**: Save the file in your package, updating the `model_path` and `image_source_topic` to match your setup.
3.  **Run the camera**: Make sure your simulated or real camera is publishing images.
4.  **Run from inside the Isaac ROS container**:
    ```bash
    # Launch the detection pipeline
    ros2 launch <your_package_name> object_detection.launch.py
    ```
5.  **Visualize**: You can now visualize the output:
    ```bash
    # View the detection messages
    ros2 topic echo /visualizer/my_image
    
    # Or use an image view to see the bounding boxes
    ros2 run image_view image_view --ros-args -r image:=/visualizer/my_image
    ```
