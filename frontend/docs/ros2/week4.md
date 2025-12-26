---
id: ros2-week4
title: "Week 4: Introduction to the ROS 2 Ecosystem"
slug: /ros2/week4
sidebar_label: "Week 4: ROS 2 Intro"
estimated_time: 4
week: 4
module: ROS 2 Ecosystem
prerequisites: ["foundations-week1", "foundations-week2", "foundations-week3"]
learning_objectives:
  - "Understand the core concepts of ROS 2 (nodes, topics, services, actions)."
  - "Learn how to create and run a ROS 2 node in Python."
  - "Use the `ros2` command-line interface to inspect a running ROS 2 system."
  - "Understand how to use launch files to start multiple nodes."
---

# Week 4: Introduction to the ROS 2 Ecosystem

This week, we transition from the theoretical foundations of robotics to the practical tools that bring them to life. We will introduce the Robot Operating System 2 (ROS 2), the open-source framework that has become the standard for robotics development.

## Topics Covered

-   **What is ROS 2?** An overview of its architecture and philosophy.
-   **ROS 2 Nodes:** The fundamental building blocks of a ROS 2 system.
-   **ROS 2 Topics:** Asynchronous, publish-subscribe communication.
-   **ROS 2 Services:** Synchronous, request-response communication.
-   **ROS 2 Actions:** Asynchronous, long-running tasks with feedback.
-   **The `ros2` CLI:** Your window into a running ROS 2 system.
-   **Launch Files:** A systematic way to start and configure your robot's software.

## What is ROS 2?

ROS 2 is a flexible framework for writing robot software. It is a set of libraries and tools that help you build complex robotics applications. Think of it as the glue that holds your robot's software together.

At its core, ROS 2 is about enabling communication between different parts of your robot's software. It allows you to create a system of small, independent programs that can communicate with each other in a standardized way. This makes it easy to build complex systems from smaller, more manageable parts.

### The ROS 2 Graph

A running ROS 2 system is a graph of interconnected nodes. A **node** is a process that performs some computation. For example, you might have a node for controlling the motors in a robot's arm, another node for reading data from a camera, and a third node for planning the robot's movements.

These nodes communicate with each other by passing **messages**. A message is a simple data structure, like an integer, a floating-point number, or a string. ROS 2 provides a rich set of predefined message types, and you can also define your own.

Nodes and messages are organized into a graph through three main communication patterns:
*   **Topics:** A node can publish messages to a topic, and any number of other nodes can subscribe to that topic to receive the messages. This is an asynchronous, one-to-many communication pattern.
*   **Services:** A node can provide a service, and another node can make a request to that service. The service provider will then do some work and send back a response. This is a synchronous, one-to-one communication pattern.
*   **Actions:** An action is similar to a service, but it is designed for long-running tasks. An action server can provide feedback to the action client while it is running, and the client can cancel the action if it is no longer needed.

## ROS 2 Nodes

A node is the fundamental building block of a ROS 2 system. It is a program that performs some computation. A well-designed ROS 2 system is composed of many small, single-purpose nodes.

### Creating a ROS 2 Node in Python

Let's create our first ROS 2 node. This node won't do much, but it will show you the basic structure of a ROS 2 node in Python.

```python
import rclpy
from rclpy.node import Node

class MyFirstNode(Node):
    def __init__(self):
        super().__init__('my_first_node')
        self.get_logger().info('Hello from my_first_node!')

def main(args=None):
    rclpy.init(args=args)

    node = MyFirstNode()

    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

Let's break down this code:
*   `import rclpy` and `from rclpy.node import Node`: These lines import the necessary ROS 2 Python libraries.
*   `class MyFirstNode(Node)`: We define a class that inherits from `rclpy.node.Node`.
*   `super().__init__('my_first_node')`: We call the parent class's constructor and give our node a name.
*   `self.get_logger().info(...)`: We use the node's logger to print a message.
*   `rclpy.init(args=args)`: This initializes the ROS 2 client library.
*   `node = MyFirstNode()`: We create an instance of our node.
*   `rclpy.spin(node)`: This is the main event loop for the node. It will keep the node running and process any incoming messages or service requests.
*   `node.destroy_node()` and `rclpy.shutdown()`: These lines clean up the node and the ROS 2 client library when the program is finished.

### Running a ROS 2 Node

To run this node, save the code as `my_first_node.py` and then run it from your terminal:

```bash
python my_first_node.py
```

You should see the "Hello from my_first_node!" message printed to your terminal.

In the next section, we will learn how to make our nodes communicate with each other using topics.

## ROS 2 Topics

Topics are the most common way for nodes to communicate in ROS 2. They use a publish/subscribe model, where one or more nodes can publish messages to a topic, and one or more nodes can subscribe to that topic to receive the messages.

This is a great way to decouple your nodes from each other. The publisher doesn't need to know who is subscribing, and the subscriber doesn't need to know who is publishing. They just need to agree on the topic name and the message type.

### Creating a Publisher

Let's create a node that publishes a simple string message to a topic called `chatter`.

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class ChatterPublisher(Node):
    def __init__(self):
        super().__init__('chatter_publisher')
        self.publisher_ = self.create_publisher(String, 'chatter', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello World: {self.i}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    chatter_publisher = ChatterPublisher()
    rclpy.spin(chatter_publisher)
    chatter_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

Here's what's new:
*   `from std_msgs.msg import String`: We import the `String` message type from the `std_msgs` package.
*   `self.create_publisher(String, 'chatter', 10)`: We create a publisher that publishes messages of type `String` to the `chatter` topic. The `10` is the queue size, which is the number of messages to buffer if the subscriber is not ready to receive them.
*   `self.create_timer(timer_period, self.timer_callback)`: We create a timer that calls the `timer_callback` method every `timer_period` seconds.
*   `self.publisher_.publish(msg)`: In the `timer_callback`, we create a `String` message, set its `data` field, and then publish it.

### Creating a Subscriber

Now, let's create a subscriber to receive the messages from our `chatter_publisher`.

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class ChatterSubscriber(Node):
    def __init__(self):
        super().__init__('chatter_subscriber')
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
    chatter_subscriber = ChatterSubscriber()
    rclpy.spin(chatter_subscriber)
    chatter_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

Here's what's new:
*   `self.create_subscription(String, 'chatter', self.listener_callback, 10)`: We create a subscription to the `chatter` topic. We specify the message type, the topic name, a callback function to be called when a message is received, and the queue size.
*   `self.listener_callback(self, msg)`: This method is called every time a message is received on the `chatter` topic. The message is passed as the `msg` argument.

### Running the Publisher and Subscriber

To run these nodes, save the code as `chatter_publisher.py` and `chatter_subscriber.py`. Then, open two terminals.

In the first terminal, run the publisher:
```bash
python chatter_publisher.py
```

In the second terminal, run the subscriber:
```bash
python chatter_subscriber.py
```

You should see the publisher printing "Publishing..." messages, and the subscriber printing "I heard..." messages.

This is the core of the ROS 2 communication system. In the next sections, we will explore the other communication patterns: services and actions.

## ROS 2 Services

Services are used for synchronous, request-response communication. A node can provide a service, and another node (a client) can send a request and wait for a response.

This is useful when you want to perform a specific action and get a result right away. For example, you might have a service that takes two numbers and returns their sum.

### Service Definition

A service is defined by a `.srv` file, which specifies the request and response message types. For example, a service to add two integers might be defined as:

```
int64 a
int64 b
---
int64 sum
```

The part above the `---` is the request, and the part below is the response.

### Creating a Service Server

A service server is a node that provides a service. It waits for requests, processes them, and sends back a response.

```python
# Pseudo-code for a service server
class AddTwoIntsServer(Node):
    def __init__(self):
        super().__init__('add_two_ints_server')
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info(f'Incoming request: a={request.a}, b={request.b}. Returning sum={response.sum}')
        return response
```

### Creating a Service Client

A service client is a node that sends a request to a service and waits for a response.

```python
# Pseudo-code for a service client
class AddTwoIntsClient(Node):
    def __init__(self):
        super().__init__('add_two_ints_client')
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        self.req = AddTwoInts.Request()

    def send_request(self):
        self.req.a = 41
        self.req.b = 1
        self.future = self.cli.call_async(self.req)
```

## ROS 2 Actions

Actions are used for asynchronous, long-running tasks. They are similar to services, but they provide feedback while the task is running and can be canceled.

This is useful for tasks that take a long time to complete, such as navigating to a goal or moving a robot arm.

### Action Definition

An action is defined by a `.action` file, which specifies the goal, result, and feedback message types.

```
# Goal
int32 order
---
# Result
int32[] sequence
---
# Feedback
int32[] partial_sequence
```

### Creating an Action Server

An action server is a node that provides an action. It waits for goals, executes them, provides feedback, and sends a result when the goal is complete.

```python
# Pseudo-code for an action server
class FibonacciActionServer(Node):
    def __init__(self):
        super().__init__('fibonacci_action_server')
        self._action_server = ActionServer(
            self,
            Fibonacci,
            'fibonacci',
            self.execute_callback)

    def execute_callback(self, goal_handle):
        # ... execute the action and provide feedback ...
        goal_handle.succeed()
        result = Fibonacci.Result()
        # ... set the result ...
        return result

## The `ros2` CLI

The `ros2` command-line interface (CLI) is your main tool for introspecting a running ROS 2 system. It is a powerful tool that allows you to see what nodes are running, what topics are being published, and much more.

Here are some of the most common `ros2` commands:

*   `ros2 node list`: Lists all the running nodes.
*   `ros2 topic list`: Lists all the active topics.
*   `ros2 topic echo <topic_name>`: Prints the messages being published on a topic.
*   `ros2 service list`: Lists all the available services.
*   `ros2 service call <service_name> <service_type> <request>`: Calls a service with a given request.
*   `ros2 action list`: Lists all the available actions.
*   `ros2 run <package_name> <executable_name>`: Runs a node from a given package.

## Launch Files

Launch files are a way to start and configure multiple ROS 2 nodes at once. They are written in Python and allow you to create complex systems of nodes with a single command.

A simple launch file might look like this:

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_package',
            executable='chatter_publisher',
            name='publisher'
        ),
        Node(
            package='my_package',
            executable='chatter_subscriber',
            name='subscriber'
        ),
    ])
```

This launch file starts two nodes: `chatter_publisher` and `chatter_subscriber`. To run this launch file, you would use the `ros2 launch` command:

```bash
ros2 launch my_package my_launch_file.py
```

This is just a brief introduction to the ROS 2 ecosystem. In the coming weeks, we will dive deeper into each of these topics and learn how to use them to build a complete robotics application.

