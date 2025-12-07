---
id: simulating-robots
title: "Week 7: Simulating Robots and Sensors"
sidebar_label: "Week 7: Robots & Sensors"
estimated_time: 5
week: 7
module: "Digital Twin"
prerequisites:
  - "intro-isaac-sim"
learning_objectives:
  - "Import a URDF robot model into Isaac Sim"
  - "Add simulated sensors, like cameras and LiDAR, to the robot"
  - "Configure the ROS 2 Bridge to publish sensor data"
  - "Visualize simulated sensor data in RViz2"
---

# Week 7: Simulating Robots and Sensors

Once you are familiar with the Isaac Sim environment, you can start bringing in your own robots and sensors. This week, we will import the URDF robot we created and add simulated sensors to it, bridging the gap between our robot model and a functioning digital twin.

## Importing a URDF

Isaac Sim has a built-in URDF Importer tool. This tool converts your URDF or XACRO file into a USD asset that can be used in the simulation. The importer will automatically handle the links, joints, and visual meshes. It also provides options for fixing joint articulation and improving physical properties.

## Adding a Camera

You can add a variety of sensors to your robot directly within the Isaac Sim UI or through Python scripting. A camera is one of the most common sensors. Once a camera is created and attached to a robot link, you can enable its ROS 2 publisher to stream the camera feed to a ROS 2 topic.

## Code Example: Spawning a Robot and a Camera

This script demonstrates how to spawn a robot from a URDF and attach a camera sensor that publishes to ROS 2.

```python
# spawn_robot_with_camera.py
import omni
from omni.isaac.core import World
from omni.isaac.core.robots import Robot
from omni.isaac.core.sensors import Camera
from omni.isaac.core.utils.nucleus import get_assets_root_path

# Assumes the URDF has been imported into a USD file on the Nucleus server
# For example: "omniverse://localhost/Projects/my_robot.usd"
assets_root_path = get_assets_root_path()
ROBOT_USD_PATH = assets_root_path + "/Isaac/Robots/Jetbot/jetbot.usd"


world = World()
world.scene.add_default_ground_plane()

# Add the robot to the stage
robot = world.scene.add(
    Robot(
        prim_path="/world/my_robot",
        name="my_robot",
        usd_path=ROBOT_USD_PATH,
        position=[0, 0, 0.1]
    )
)

# Add a camera to the robot's chassis link
camera = world.scene.add(
    Camera(
        prim_path="/world/my_robot/chassis_link/camera",
        name="my_camera",
        position=[0.2, 0, 0.5], # Position relative to the chassis link
        frequency=30,
        resolution=(640, 480),
    )
)

# Enable ROS 2 bridge to publish camera data
# This assumes the ROS 2 bridge extension is enabled
camera.add_ros_camera_bridge("image_raw")


world.reset()
while True:
    world.step(render=True)
```

### How to Run

1.  First, use the **URDF Importer** in Isaac Sim (`Window` -> `Importers` -> `URDF Importer`) to convert your robot's XACRO/URDF file into a USD file. Save this file to your local Nucleus server. Update the `ROBOT_USD_PATH` in the script to point to your new file.
2.  Make sure the `omni.isaac.ros2_bridge` extension is enabled in Isaac Sim (`Window` -> `Extensions`).
3.  Run the script from the Script Editor.
4.  In a ROS 2 terminal, you can now see the camera data:
    ```bash
    # See the new topic
    ros2 topic list
    
    # View the camera feed
    ros2 run image_view image_view --ros-args -r /image_raw:=/world/my_robot/chassis_link/camera/image_raw
    ```
