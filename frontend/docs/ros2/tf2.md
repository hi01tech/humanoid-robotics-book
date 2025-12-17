---
id: tf2
title: 'Module 1: Coordinate Frames with tf2'
sidebar_label: 'tf2'
---

## Handling Coordinate Frames with tf2

In any robot with moving parts or multiple sensors, you need a way to keep track of how all the different coordinate frames relate to each other. In ROS 2, this is handled by the `tf2` library. `tf2` allows you to ask questions like, "What is the position of the robot's hand relative to its base?" or "What is the pose of the object detected by the camera in the map frame?"

### Key Concepts of tf2

*   **Transform (tf):** A transform represents the relationship between two coordinate frames. It includes both a translation (a 3D vector) and a rotation (a quaternion).
*   **tf2 Buffer:** A buffer stores transforms received from the `/tf` and `/tf_static` topics.
*   **tf2 Listener:** A listener subscribes to the transform topics and populates a `tf2` buffer.
*   **tf2 Broadcaster:** A broadcaster publishes transforms to the `/tf` and `/tf_static` topics.

### Example: A Static Transform Publisher

Often, some parts of a robot are fixed relative to others. For example, a camera might be mounted at a specific position on the robot's chassis. You can publish this relationship as a "static transform."

Here is how you can publish a static transform using a Python script. This script publishes a transform between a `base_link` frame and a `camera_link` frame.

```python
# static_tf_pub.py
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
import tf_transformations

class StaticFramePublisher(Node):
    """
    Broadcasts a static transform.
    """
    def __init__(self):
        super().__init__('static_transform_publisher')
        self._broadcaster = StaticTransformBroadcaster(self)

        # Publish a static transform once at startup
        self.make_transforms()

    def make_transforms(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'base_link'
        t.child_frame_id = 'camera_link'

        # The camera is positioned 1 meter forward, 0.5 meters up
        t.transform.translation.x = 1.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.5

        # No rotation for this example
        q = tf_transformations.quaternion_from_euler(0, 0, 0)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        self.get_logger().info(f"Publishing static transform from '{t.header.frame_id}' to '{t.child_frame_id}'")
        self._broadcaster.sendTransform(t)

def main():
    rclpy.init()
    node = StaticFramePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```
To run this, you will need the `tf_transformations` library. You can install it with `pip install tf-transformations`.

### Visualizing with RViz2

The best way to understand `tf2` is to visualize it.
1.  Run the `static_tf_pub.py` node.
2.  Open RViz2 by running `rviz2` in a new terminal.
3.  In the "Displays" panel, click "Add" and select "TF".
4.  You should now see the `base_link` and `camera_link` frames visualized, showing their relative positions and orientations.

This provides a powerful way to debug and understand the spatial relationships within your robot system. We will use `tf2` extensively when we work with robot models and navigation.