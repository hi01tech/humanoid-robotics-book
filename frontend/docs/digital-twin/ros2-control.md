---
id: ros2-control
title: 'Module 2: Using ros2_control'
sidebar_label: 'ros2_control'
---

## Real-time Control with `ros2_control`

While publishing joint commands directly to Isaac Sim works for simple cases, a more robust and standard approach for robot control in ROS 2 is using the `ros2_control` framework. `ros2_control` is a set of packages that provides a standardized way to interface with robot hardware. By using it with our digital twin, we create a seamless transition from simulation to a physical robot.

### Why use `ros2_control`?

The `ros2_control` framework provides several key advantages:
*   **Standardization:** It provides a common interface for controllers (like joint trajectory controllers) and hardware (or simulation) backends.
*   **Real-time Safety:** It is designed with real-time performance and safety in mind, which is critical for physical robots.
*   **Controller Management:** It includes a controller manager that can load, unload, start, and stop different controllers at runtime without restarting the robot.
*   **Abstraction:** Your high-level code (e.g., a navigation stack) doesn't need to know the details of how the robot's joints are controlled. It just sends a standard goal (like a trajectory) to a `ros2_control` controller.

### The `ros2_control` Architecture

The main components of `ros2_control` are:
1.  **Hardware Interface:** This is the component that communicates directly with the robot's hardware (or, in our case, the simulator). It reads sensor data (like joint positions) and writes commands (like joint velocities). For Isaac Sim, there are specialized hardware interface plugins.
2.  **Controller Manager:** A node that manages the lifecycle of controllers.
3.  **Controllers:** These are the algorithms that compute the commands to be sent to the hardware. Common controllers include:
    *   `joint_state_broadcaster`: Reads the current state of the joints from the hardware interface and publishes them as `sensor_msgs/msg/JointState`.
    *   `joint_trajectory_controller`: Accepts a trajectory of joint positions and executes it on the robot.
    *   `diff_drive_controller`: For controlling differential drive mobile robots.

### `ros2_control` with Isaac Sim

NVIDIA provides a `ros2_control` hardware interface for Isaac Sim. This allows `ros2_control` to treat the simulated robot in Isaac Sim as its hardware backend.

Here is the typical workflow:
1.  **URDF Configuration:** Your robot's URDF is updated to include special `<ros2_control>` tags. These tags define the available command interfaces (e.g., `position`, `velocity`) and state interfaces for each joint.

    ```xml
    <ros2_control name="IsaacSimSystem" type="system">
      <hardware>
        <plugin>isaac_sim_ros2_control/IsaacSimROS2ControlHardware</plugin>
        <param name="robot_description_topic">/robot_description</param>
      </hardware>
      <joint name="shoulder_joint">
        <command_interface name="position"/>
        <state_interface name="position"/>
        <state_interface name="velocity"/>
      </joint>
    </ros2_control>
    ```

2.  **Controller Configuration:** You create a YAML file to configure the controllers you want to use.

    ```yaml
    controller_manager:
      ros__parameters:
        update_rate: 100
        use_sim_time: true

        joint_state_broadcaster:
          type: joint_state_broadcaster/JointStateBroadcaster

        joint_trajectory_controller:
          type: joint_trajectory_controller/JointTrajectoryController

    joint_trajectory_controller:
      ros__parameters:
        joints:
          - shoulder_joint
        command_interfaces:
          - position
        state_interfaces:
          - position
          - velocity
    ```

3.  **Launch:** A launch file is used to:
    *   Load the robot description (URDF).
    *   Start the `ros2_control` controller manager (`ros2_control_node`).
    *   Load and start the controllers defined in the YAML file.

### Sending a Goal to a Controller

With `ros2_control` set up, you no longer publish low-level joint commands. Instead, you send a high-level goal to the appropriate controller. For the `joint_trajectory_controller`, this is an "action" of type `control_msgs/action/FollowJointTrajectory`.

Here is a simplified Python example of sending a trajectory goal to move the `shoulder_joint`.

```python
# trajectory_goal_sender.py
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

class TrajectoryClient(Node):
    def __init__(self):
        super().__init__('trajectory_client')
        self._action_client = ActionClient(self, FollowJointTrajectory, '/joint_trajectory_controller/follow_joint_trajectory')

    def send_goal(self):
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = ['shoulder_joint']
        
        point = JointTrajectoryPoint()
        point.positions = [0.5] # Target position in radians
        point.time_from_start.sec = 2
        goal_msg.trajectory.points.append(point)

        self.get_logger().info('Sending goal...')
        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(goal_msg)

def main(args=None):
    rclpy.init(args=args)
    action_client = TrajectoryClient()
    action_client.send_goal()
    rclpy.spin(action_client) # Keep node alive to handle action feedback/result
    action_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```
This approach is much more powerful. You can specify a sequence of points with velocities and accelerations, and the controller will handle the smooth execution of the trajectory. This is how complex arm movements are orchestrated in ROS 2.