---
id: ros2-week5
title: "Week 5: Transforms and URDFs in ROS 2"
slug: /ros2/week5
sidebar_label: "Week 5: Transforms & URDFs"
estimated_time: 4
week: 5
module: ROS 2 Ecosystem
prerequisites: ["ros2-week4"]
learning_objectives:
  - "Understand the importance of coordinate frames in robotics."
  - "Use `tf2` to broadcast and listen to coordinate transformations."
  - "Create a Unified Robot Description Format (URDF) file for a simple robot."
  - "Visualize a robot model in RViz2."
---

# Week 5: Transforms and URDFs in ROS 2

This week, we will tackle two of the most important concepts in ROS 2: coordinate transformations (`tf2`) and robot descriptions (URDF). These are the tools that allow us to represent and track the state of a robot in a 3D world.

## Topics Covered

-   **Coordinate Frames:** Why we need them and how they work.
-   **The `tf2` Library:** Broadcasting and listening to transforms.
-   **Static and Dynamic Transforms:** Representing fixed and moving parts of a robot.
-   **Unified Robot Description Format (URDF):** Describing a robot's physical properties.
-   **RViz2:** Visualizing robot models and `tf2` frames.

## Coordinate Frames

In robotics, we are always dealing with objects in a 3D world. To describe the position and orientation of these objects, we use **coordinate frames**. A coordinate frame is a set of three orthogonal axes (x, y, and z) that define a coordinate system.

A robot is a collection of different parts, each with its own coordinate frame. For example, a humanoid robot might have a coordinate frame for its base, another for its torso, one for each arm, and one for each leg. On top of that, you might have a coordinate frame for a camera, a Lidar, and any other sensors on the robot.

The `tf2` library in ROS 2 is a tool for keeping track of all these different coordinate frames and the transformations between them.

## The `tf2` Library

`tf2` is a powerful library that lets you keep track of multiple coordinate frames over time. It maintains a tree of coordinate frames and allows you to ask for the transformation between any two frames at any point in time.

### Broadcasting Transforms

A **transform** is a set of instructions that tells you how to get from one coordinate frame to another. It consists of a translation (a 3D vector) and a rotation (a quaternion).

A `tf2` **broadcaster** is a node that publishes transforms. For example, you might have a node that reads the joint angles of a robot arm and publishes the transforms for each link in the arm.

Here is a simple example of a `tf2` broadcaster that publishes a static transform between a `world` frame and a `robot` frame:

```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

class StaticFramePublisher(Node):
    """
    Publishes a static transform between the 'world' and 'robot' frames.
    """
    def __init__(self):
        super().__init__('static_transform_publisher')
        self._tf_publisher = StaticTransformBroadcaster(self)

        # Publish a static transform once on startup
        self.make_transforms()

    def make_transforms(self):
        t = TransformStamped()

        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'world'
        t.child_frame_id = 'robot'

        t.transform.translation.x = 1.0
        t.transform.translation.y = 2.0
        t.transform.translation.z = 0.0

        # No rotation in this example
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0

        self._tf_publisher.sendTransform(t)

def main():
    rclpy.init()
    node = StaticFramePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Listening for Transforms

A `tf2` **listener** is a node that subscribes to the `/tf` topic and can be used to get the transformation between any two coordinate frames.

For example, if you have a Lidar on your robot, you might want to transform the Lidar data from the `lidar` frame to the `world` frame to see where the obstacles are in the world.

We will see examples of `tf2` listeners in later weeks when we start working with sensor data.


## Unified Robot Description Format (URDF)

A URDF file is an XML file that describes the physical properties of a robot. It is a tree of **links** and **joints**.

*   A **link** is a rigid part of the robot, like a a wheel, a leg, or a torso. Each link has a name and can have visual and collision properties.
*   A **joint** connects two links and defines how they can move relative to each other. A joint can be `revolute` (for a rotating joint), `prismatic` (for a sliding joint), `fixed` (for a joint that doesn't move), or `continuous` (for a joint that can rotate continuously).

Here is a simple example of a URDF file for a two-wheeled robot:

```xml
<?xml version="1.0"?>
<robot name="my_robot">

  <link name="base_link">
    <visual>
      <geometry>
        <cylinder length="0.1" radius="0.2"/>
      </geometry>
    </visual>
  </link>

  <link name="left_wheel">
    <visual>
      <geometry>
        <cylinder length="0.05" radius="0.1"/>
      </geometry>
    </visual>
  </link>

  <joint name="base_to_left_wheel" type="continuous">
    <parent link="base_link"/>
    <child link="left_wheel"/>
    <origin xyz="0 0.25 -0.05"/>
    <axis xyz="0 1 0"/>
  </joint>

</robot>
```

This URDF defines a robot with a `base_link` and a `left_wheel`, connected by a `continuous` joint.

## RViz2

RViz2 is a powerful 3D visualization tool for ROS 2. It allows you to see what your robot is doing and to debug your robotics application.

With RViz2, you can:
*   Visualize your robot model (from a URDF file).
*   Display `tf2` frames to see the relationship between different coordinate frames.
*   Visualize sensor data, such as Lidar scans, camera images, and point clouds.
*   Display the output of your navigation and manipulation algorithms.

RViz2 is an essential tool for any ROS 2 developer, and we will be using it extensively throughout this course.

