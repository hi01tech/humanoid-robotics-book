---
id: ros2-launch-params
title: "Week 5: Launch Files & Parameters"
sidebar_label: "Week 5: Launch & Params"
estimated_time: 3
week: 5
module: "ROS 2"
prerequisites:
  - "ros2-core-concepts"
learning_objectives:
  - "Understand the purpose of ROS 2 launch files"
  - "Write a Python-based launch file to start multiple nodes"
  - "Use parameters to make nodes configurable"
  - "Set and read a parameter from a launch file"
---

# Week 5: Launch Files & Parameters

As your robotics projects grow, manually running each node in a separate terminal becomes tedious. ROS 2 **Launch Files** solve this by allowing you to define and run a complex system of nodes with a single command. **Parameters** add another layer of flexibility, letting you configure your nodes externally without changing their code.

## Python Launch Files

While you can use XML or YAML, Python launch files are the most powerful and flexible. They allow you to use programming logic to define your system startup.

A launch file typically includes:
-   A `generate_launch_description()` function.
-   A list of `Node` actions to execute.

## Parameters

Parameters are values (integers, strings, booleans, etc.) associated with a node. You can set them from a launch file, a YAML file, or the command line. Inside your node, you can declare parameters and retrieve their values.

## Code Examples

### 1. Parameter Node

This node declares a parameter `my_parameter` and prints its value.

```python
# param_node.py
import rclpy
from rclpy.node import Node

class ParamNode(Node):
    def __init__(self):
        super().__init__('param_node')
        # Declare the parameter with a default value
        self.declare_parameter('my_parameter', 'world')
        
        # Create a timer to read and print the parameter value
        self.timer = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        my_param = self.get_parameter('my_parameter').get_parameter_value().string_value
        self.get_logger().info(f'Hello, {my_param}!')

def main(args=None):
    rclpy.init(args=args)
    node = ParamNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 2. Launch File

This launch file starts the `param_node` and sets `my_parameter` to "Earth".

```python
# my_launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='<your_package_name>',
            executable='param_node',
            name='my_param_node',
            output='screen',
            parameters=[{'my_parameter': 'Earth'}]
        )
    ])
```

### How to Run

1.  Save both files in your ROS 2 package (the launch file in a `launch` directory).
2.  Add the launch file installation to your `setup.py`.
3.  Build your package with `colcon build`.
4.  Run the launch file: `ros2 launch <your_package_name> my_launch.py`. You should see the message "Hello, Earth!" printed every second.
