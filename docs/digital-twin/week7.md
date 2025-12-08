---
id: week7
title: 'Module 2: Interacting with the Digital Twin'
sidebar_label: 'Week 7: ROS 2 Interaction'
---

## Week 7: Interacting with the Digital Twin via ROS 2

Last week, we introduced the concept of a digital twin. This week, we'll dive into the practical aspects of communicating with our digital twin in Isaac Sim using ROS 2. Isaac Sim comes with a powerful set of ROS 2 bridge extensions that make this communication seamless.

### The ROS 2 Bridge in Isaac Sim

The "ROS 2 Bridge" is an extension within Isaac Sim that connects the simulation environment to the ROS 2 network. It can:
*   **Publish simulation data to ROS 2 topics:** This includes sensor data (like camera images, LiDAR scans, and IMU readings), the state of the robot (joint states), and simulation clock.
*   **Subscribe to ROS 2 topics to control the simulation:** This allows us to send commands from our ROS 2 nodes to control robot joints, apply forces, and change the environment.

This means that from the perspective of our ROS 2 code, there is no difference between interacting with the simulated robot and a physical one. We just publish and subscribe to the same topics.

### Example: Reading Sensor Data from the Digital Twin

Let's imagine our digital twin has a camera attached to it. We can configure Isaac Sim to publish the images from this camera to a ROS 2 topic.

#### In Isaac Sim:
1.  Add a camera to the robot model in the Isaac Sim scene.
2.  Add the "ROS 2 Camera Helper" graph node to the camera's action graph.
3.  Configure the node to publish the camera data to a topic, for example, `/robot/camera/image_raw`.

#### In ROS 2 (Python Node):
We can then write a simple subscriber node to receive and process these images. This example uses `cv_bridge` to convert the ROS 2 image message to an OpenCV image, which is a common practice.

```python
# image_subscriber.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber')
        self.subscription = self.create_subscription(
            Image,
            '/robot/camera/image_raw',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.bridge = CvBridge()
        self.get_logger().info("Image subscriber node started and subscribing to '/robot/camera/image_raw'")

    def listener_callback(self, data):
        self.get_logger().info('Receiving video frame')
        # Convert ROS Image message to OpenCV image
        current_frame = self.bridge.imgmsg_to_cv2(data)
        
        # In a real application, you would perform image processing here.
        # For this example, we'll just display the image.
        cv2.imshow("Camera Feed", current_frame)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    image_subscriber = ImageSubscriber()
    rclpy.spin(image_subscriber)
    image_subscriber.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
```
To run this, you'll need `opencv-python` and `cv_bridge`. You can typically install OpenCV with `pip install opencv-python`. `cv_bridge` is a standard ROS package.

### Example: Sending Commands to the Digital Twin

The communication flows both ways. We can also send commands to our digital twin. A common task is to control the joints of a robot arm.

#### In Isaac Sim:
1.  Ensure your robot's joints are correctly configured.
2.  Add a "ROS 2 Joint State" subscriber node to the action graph. This node will listen for `sensor_msgs/msg/JointState` messages.
3.  This node will drive the robot's joints based on the positions received in the messages.

#### In ROS 2 (Python Node):
We can write a publisher node that sends joint commands. This example sends a command to move a "shoulder_joint" to a specific position.

```python
# joint_commander.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class JointCommander(Node):
    def __init__(self):
        super().__init__('joint_commander')
        self.publisher_ = self.create_publisher(JointState, '/joint_command', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.target_position = 0.785  # 45 degrees in radians

    def timer_callback(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['shoulder_joint']
        msg.position = [self.target_position]
        
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing joint command for shoulder_joint: {self.target_position}')
        
        # Toggle position for demonstration
        if self.target_position > 0:
            self.target_position = -0.785
        else:
            self.target_position = 0.785

def main(args=None):
    rclpy.init(args=args)
    joint_commander = JointCommander()
    rclpy.spin(joint_commander)
    joint_commander.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```
When this node is running, the `shoulder_joint` of the robot in Isaac Sim will move back and forth between 45 and -45 degrees. This demonstrates direct control of the digital twin from our external ROS 2 code.