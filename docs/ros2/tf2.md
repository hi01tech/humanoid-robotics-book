---
id: ros2-tf2
title: "Coordinate Transformations with tf2"
sidebar_label: "tf2 Transformations"
estimated_time: 4
week: 5 # Part of week 5
module: "ROS 2"
prerequisites:
  - "ros2-core-concepts"
learning_objectives:
  - "Understand why coordinate frame transformations are critical in robotics"
  - "Use tf2 to broadcast a coordinate transform"
  - "Use tf2 to listen for and use a coordinate transform"
  - "Differentiate between static and dynamic transforms"
---

# Coordinate Transformations with tf2

A robot is made of many parts (links, sensors, grippers), and operates in an environment (a map, a room). `tf2` is the ROS 2 library that lets you keep track of all these different coordinate frames over time. It lets you ask questions like, "What is the position of the gripper relative to the robot's base?" or "Where is the detected obstacle in the map frame?"

## The Transform Tree

`tf2` builds a tree of coordinate frames. Each frame has a parent, and `tf2` can compute the transformation between any two frames in the tree, as long as they are connected.

## Broadcasters and Listeners

-   A **TransformBroadcaster** is used to send out (broadcast) transforms. You would typically have a broadcaster in any node that is publishing data from a sensor or controlling a moving part of the robot.
-   A **TransformListener** subscribes to the transform data and provides an interface to query the relationship between frames.

## Code Examples

### 1. Transform Broadcaster

This node broadcasts a transform from `base_link` to `moving_frame` that rotates and moves over time.

```python
# tf_broadcaster.py
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import tf_transformations
import math

class DynamicTFBroadcaster(Node):
    def __init__(self):
        super().__init__('dynamic_tf_broadcaster')
        self.br = TransformBroadcaster(self)
        self.timer = self.create_timer(0.1, self.broadcast_timer_callback)

    def broadcast_timer_callback(self):
        t = TransformStamped()
        
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'base_link'
        t.child_frame_id = 'moving_frame'
        
        # Make the frame move in a circle
        x = self.get_clock().now().nanoseconds / 1e9 * 0.5
        t.transform.translation.x = math.sin(x) * 2.0
        t.transform.translation.y = math.cos(x) * 2.0
        t.transform.translation.z = 0.0
        
        q = tf_transformations.quaternion_from_euler(0, 0, x)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        
        self.br.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = DynamicTFBroadcaster()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 2. Transform Listener

This node creates a point and uses the transform from the broadcaster to see where that point is in the `base_link` frame.

```python
# tf_listener.py
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from geometry_msgs.msg import PointStamped

class TFListener(Node):
    def __init__(self):
        super().__init__('tf_listener')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.timer = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        try:
            # Get the transform from 'moving_frame' to 'base_link'
            trans = self.tf_buffer.lookup_transform(
                'base_link', 'moving_frame', rclpy.time.Time(), timeout=Duration(seconds=0.1))

            self.get_logger().info(
                f'Transform is: T=({trans.transform.translation.x:.2f}, '
                f'{trans.transform.translation.y:.2f}, {trans.transform.translation.z:.2f}) '
                f'R=({trans.transform.rotation.x:.2f}, {trans.transform.rotation.y:.2f}, '
                f'{trans.transform.rotation.z:.2f}, {trans.transform.rotation.w:.2f})'
            )
        except Exception as e:
            self.get_logger().warn(f'Could not transform: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = TFListener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### How to Run

1.  Make sure you have `tf_transformations` installed (`pip install tf-transformations`).
2.  Add a static transform from `world` to `base_link` so the tree is complete: `ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 world base_link`
3.  Run the broadcaster: `ros2 run <your_package_name> tf_broadcaster`
4.  Run the listener: `ros2 run <your_package_name> tf_listener`
5.  View the transforms in RViz2 by adding the "TF" display.
