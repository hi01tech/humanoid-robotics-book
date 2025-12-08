---
id: week5
title: 'Module 1: ROS 2 Services & Actions'
sidebar_label: 'Week 5: Services & Actions'
---

## Week 5: ROS 2 Services and Actions

This week, we expand on our ROS 2 knowledge by exploring two additional communication patterns: Services and Actions. These are essential for request/response and long-running tasks in robotics.

### ROS 2 Services

Services are used for synchronous request/response communication. A "service client" sends a request to a "service server," which processes the request and sends back a response. This is useful for tasks that should be completed quickly and will always result in a response. For example, a service could be used to query the current position of a robot's arm.

Let's create a simple service that adds two integers.

#### Service Definition (`.srv` file)
First, we need to define the service interface in a `.srv` file. Let's call it `AddTwoInts.srv`.

```
int64 a
int64 b
---
int64 sum
```
The part above the `---` is the request, and the part below is the response.

#### Service Server Node

Here is the Python code for the service server.

```python
# add_two_ints_server.py
from example_interfaces.srv import AddTwoInts
import rclpy
from rclpy.node import Node

class MinimalService(Node):
    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info(f'Incoming request\na: {request.a} b: {request.b}')
        self.get_logger().info(f'Sending back response: [{response.sum}]')
        return response

def main(args=None):
    rclpy.init(args=args)
    minimal_service = MinimalService()
    rclpy.spin(minimal_service)
    minimal_service.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

#### Service Client Node

Here is the Python code for the service client that calls the server.

```python
# add_two_ints_client.py
from example_interfaces.srv import AddTwoInts
import rclpy
from rclpy.node import Node

class MinimalClientAsync(Node):
    def __init__(self):
        super().__init__('minimal_client_async')
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = AddTwoInts.Request()

    def send_request(self, a, b):
        self.req.a = a
        self.req.b = b
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

def main(args=None):
    rclpy.init(args=args)
    minimal_client = MinimalClientAsync()
    response = minimal_client.send_request(5, 10)
    minimal_client.get_logger().info(
        f'Result of add_two_ints: for 5 + 10 = {response.sum}')
    minimal_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### ROS 2 Actions

Actions are used for long-running tasks that provide feedback during execution. Think of navigating a robot to a goal position. This task might take a while, and you'd want to receive updates on its progress. Actions consist of a goal, feedback, and a result.

An "action client" sends a goal to an "action server". The server executes the goal, provides periodic feedback to the client, and sends a final result when the task is complete. This is an asynchronous process.

Due to the complexity of a full action example, we will revisit a practical implementation in a later module when we work with navigation. The core concept to remember is that actions are for tasks that take time and where you need to know what's happening during execution.