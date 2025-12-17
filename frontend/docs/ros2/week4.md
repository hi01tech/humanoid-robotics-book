---
id: week4
title: 'Module 1: ROS 2 Concepts'
sidebar_label: 'Week 4: Core Concepts'
---

## Week 4: Diving into ROS 2 Core Concepts

This week, we will explore the fundamental concepts of ROS 2 that form the backbone of any robotics application. We will cover Nodes, Topics, Services, and Actions.

### ROS 2 Nodes

A ROS 2 node is an executable that performs a specific task. In a complex robotics system, you'll have many nodes, each responsible for a small part of the overall functionality. For example, you might have one node for controlling the wheel motors, another for reading laser scan data, and another for path planning.

Here is a simple example of a ROS 2 node written in Python. This node, `simple_publisher`, creates a publisher that sends string messages on a topic named `chatter`.

```python
# simple_publisher.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SimplePublisher(Node):
    def __init__(self):
        super().__init__('simple_publisher')
        self.publisher_ = self.create_publisher(String, 'chatter', 10)
        self.timer = self.create_timer(0.5, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello from ROS 2: {self.i}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    simple_publisher = SimplePublisher()
    rclpy.spin(simple_publisher)
    simple_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### ROS 2 Topics

Topics are named buses over which nodes exchange messages. A node can publish messages to a topic (a publisher) or subscribe to a topic to receive messages (a subscriber). This provides a decoupled way for different parts of your system to communicate.

Here's a subscriber node that listens to the `chatter` topic from our `simple_publisher` example.

```python
# simple_subscriber.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SimpleSubscriber(Node):
    def __init__(self):
        super().__init__('simple_subscriber')
        self.subscription = self.create_subscription(
            String,
            'chatter',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info(f'I heard: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    simple_subscriber = SimpleSubscriber()
    rclpy.spin(simple_subscriber)
    simple_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Running the Example

To run this publisher/subscriber example:
1.  Save the code for the publisher as `simple_publisher.py` and the subscriber as `simple_subscriber.py`.
2.  Source your ROS 2 Iron setup file: `source /opt/ros/iron/setup.bash`
3.  In one terminal, run the publisher: `python3 simple_publisher.py`
4.  In another terminal, run the subscriber: `python3 simple_subscriber.py`

You will see the publisher sending messages and the subscriber receiving them. This simple example demonstrates the core power of ROS 2's topic-based communication.