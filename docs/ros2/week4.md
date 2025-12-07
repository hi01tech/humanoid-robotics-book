--- 
id: ros2-core-concepts
title: "Week 4: Core ROS 2 Concepts"
sidebar_label: "Week 4: Core Concepts"
estimated_time: 4
week: 4
module: "ROS 2"
prerequisites:
  - "foundations-week3"
learning_objectives:
  - "Understand the role of Publishers and Subscribers"
  - "Implement a ROS 2 Service Server and Client"
  - "Understand ROS 2 Actions for long-running tasks"
  - "Write a single Python node that combines multiple communication types"
---

# Week 4: Core ROS 2 Concepts

Building on our introduction last week, this week we dive deep into the fundamental communication patterns that power ROS 2. Understanding these concepts is crucial for building any robotics application.

## Publishers and Subscribers

The most common communication method in ROS 2 is the publish/subscribe model. Nodes **publish** messages on named **topics**, and other nodes can **subscribe** to these topics to receive the messages. This is a one-to-many communication pattern that decouples data producers from consumers.

## Services

Services are used for request/response communication. A node offers a **service**, and another node can act as a **client** to call that service. Unlike topics, services are synchronous: the client sends a request and waits for a response from the server. This is ideal for tasks that should be completed before the program continues.

## Actions

Actions are for long-running, asynchronous tasks that provide feedback. Think of commanding a robot to navigate to a goal. The task might take a long time, and you want to receive updates on its progress and have the ability to cancel it. An **action server** provides the goal-oriented task, and an **action client** sends goals to it.

## Code Example: Multi-Communication Node

This example demonstrates all three communication types in a single Python node.

```python
# multi_comm_node.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from example_interfaces.srv import AddTwoInts
import time

class MultiCommNode(Node):
    def __init__(self):
        super().__init__('multi_comm_node')
        
        # Publisher
        self.publisher_ = self.create_publisher(String, 'chatter', 10)
        self.timer = self.create_timer(1.0, self.publish_message)
        self.i = 0

        # Subscriber
        self.subscription = self.create_subscription(
            String,
            'chatter',
            self.receive_message,
            10)
        
        # Service Server
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)

        # Service Client
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = AddTwoInts.Request()

    def publish_message(self):
        msg = String()
        msg.data = f'Hello World: {self.i}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1

    def receive_message(self, msg):
        self.get_logger().info(f'I heard: "{msg.data}"')

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info(f'Incoming request\na: {request.a} b: {request.b}')
        self.get_logger().info(f'Returning: {response.sum}')
        return response

    def send_request(self, a, b):
        self.req.a = a
        self.req.b = b
        return self.cli.call_async(self.req)

def main(args=None):
    rclpy.init(args=args)
    node = MultiCommNode()
    
    # Example of calling the service client
    future = node.send_request(2, 3)
    rclpy.spin_until_future_complete(node, future)
    try:
        response = future.result()
        node.get_logger().info(f'Result of add_two_ints: {response.sum}')
    except Exception as e:
        node.get_logger().error(f'Service call failed {e!r}')

    rclpy.spin(node)
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### How to Run

1.  Save the code as `multi_comm_node.py` in your ROS 2 package.
2.  Build your package with `colcon build`.
3.  Run the node with `ros2 run <your_package_name> multi_comm_node`.
