---
id: week9
title: 'Module 3: Robots and ROS 2 in Isaac Sim'
sidebar_label: 'Week 9: Importing Robots'
---

## Week 9: Importing and Controlling Robots with ROS 2

Last week, we familiarized ourselves with the Isaac Sim environment. This week, our goal is to bring our robot into the simulation and establish the ROS 2 communication bridge we've been discussing.

### Importing a URDF

The first step in creating our digital twin is to import the robot's description. Isaac Sim has a built-in URDF importer that converts a URDF file into the Universal Scene Description (USD) format, which is the native format for Omniverse.

The importer tool can be found under `Window > Isaac > Importer > URDF`. When you import your URDF, the tool will:
*   Parse the `<link>` and `<joint>` structure.
*   Create a USD prim for each link and joint.
*   Assign physics properties based on the `<inertial>` and `<collision>` tags.
*   Set up the joint types and limits.

It's crucial to have a well-defined URDF with accurate collision meshes and inertial properties for the simulation to be stable and realistic.

#### The URDF Import Process
1.  Open the URDF Importer.
2.  Select your input `.urdf` file.
3.  Choose an output directory where the converted USD assets will be saved.
4.  Click "Import".

After the import, you will have a new USD file representing your robot. You can drag this file from the Content Browser into your scene to add your robot to the world.

### Setting up the ROS 2 Bridge

Once the robot is in the scene, the next step is to enable ROS 2 communication. Isaac Sim provides a set of tools, primarily through its "Action Graph" visual scripting interface, to connect simulation elements to ROS 2 topics.

Here's a standard setup for a robot arm or a humanoid:

1.  **Select the Robot Prim:** In the Stage, find the root prim of your imported robot.
2.  **Create an Action Graph:** In the Property panel for the robot prim, click `+ Add > Graph > Action Graph`.
3.  **Open the Action Graph Editor:** Click the "Edit" button next to the newly created graph.

Inside the Action Graph, you will add nodes to handle ROS 2 communication:

*   **`On Playback Tick` Node:** This node fires on every simulation step, driving the execution of the graph.
*   **`ROS2 Context` Node:** Establishes the connection to the ROS 2 domain.
*   **`ROS2 Subscribe JointState` Node:** This node subscribes to a topic (e.g., `/joint_command`) of type `sensor_msgs/msg/JointState`. The received joint positions will be used to drive the robot's joints.
*   **`Articulation Kinematics` Node:** This is the node that takes the target joint positions from the subscriber and applies them to the robot's articulation (the physics representation of the robot's joints).
*   **`ROS2 Publish JointState` Node:** This node reads the current state of the robot's joints from the simulation and publishes them to a topic (e.g., `/joint_states`). This is crucial for `robot_state_publisher` and other ROS 2 nodes that need to know the robot's current configuration.
*   **`ROS2 Publish Clock` Node:** Publishes the simulation time to the `/clock` topic. This is essential for ROS 2 nodes to operate in sync with the simulation. When using a simulator, you should always set the `use_sim_time` parameter to `true` in your ROS 2 nodes.

Connecting these nodes in the Action Graph creates a complete ROS 2 bridge for your robot. The `ROS2 Subscribe JointState` node receives commands, and the `ROS2 Publish JointState` node sends back the robot's status.

### Example: A ROS 2 Bridge Launch Script

While you can set up the bridge manually using the UI, the real power of Isaac Sim comes from scripting. You can write a Python script that automates the entire process: loading the robot, creating the world, and setting up the ROS 2 bridge.

```python
# ros_bridge_setup.py
from omni.isaac.core.tasks import BaseTask
from omni.isaac.core.articulations import Articulation
from omni.isaac.core.utils.prims import get_prim_at_path
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.utils.viewports import set_camera_view
from omni.isaac.ros2.scripts.context import ROS2Context
from omni.isaac.ros2.scripts.articulation import ROS2Articulation

class RobotROS2BridgeTask(BaseTask):
    def __init__(self, name):
        super().__init__(name=name, offset=None)
        # Path to your robot's USD file
        self._robot_usd_path = "/path/to/your/robot.usd"
        # ROS 2 context (must be initialized before other ROS 2 nodes)
        self.ros2_context = ROS2Context()

    def set_up_scene(self, scene):
        super().set_up_scene(scene)
        # Add the robot to the stage
        add_reference_to_stage(self._robot_usd_path, "/World/Robot")
        # Get the articulation wrapper for the robot
        self._robot_articulation = Articulation("/World/Robot")
        scene.add(self._robot_articulation)
        
        # Initialize the ROS 2 Articulation wrapper
        # This automatically sets up publishers and subscribers
        self.ros2_articulation = ROS2Articulation(
            articulation_path="/World/Robot",
            node_name="robot_ros2_interface",
            publish_joint_state=True,
            subscribe_joint_command=True
        )
        
        self.get_logger().info("ROS 2 bridge for robot initialized.")
        return

    def post_reset(self):
        # Set camera view to focus on the robot
        set_camera_view(eye=[2.0, 2.0, 2.0], target=[0, 0, 0.5])

# In your main script execution:
# 1. Initialize the simulation world
# world = World(stage_units_in_meters=1.0)
# 2. Add the task to the world's task manager
# task = RobotROS2BridgeTask(name="robot_task")
# world.add_task(task)
# 3. Reset the world to run the setup
# world.reset()
# 4. Loop the simulation
# while simulation_app.is_running():
#     world.step(render=True)
```
This script demonstrates the "code-first" approach to using Isaac Sim, which is ideal for creating repeatable and automated simulation environments. With this setup, you can now run your ROS 2 nodes from Module 1 and 2, and they will seamlessly control and monitor your robot in Isaac Sim.