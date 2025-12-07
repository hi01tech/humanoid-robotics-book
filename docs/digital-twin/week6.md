---
id: intro-isaac-sim
title: "Week 6: Introduction to Isaac Sim"
sidebar_label: "Week 6: Intro to Isaac Sim"
estimated_time: 4
week: 6
module: "Digital Twin"
prerequisites:
  - "ros2-urdf"
learning_objectives:
  - "Understand the role of Isaac Sim as a robotics simulator"
  - "Navigate the Isaac Sim user interface and viewport"
  - "Understand the concept of a USD (Universal Scene Description)"
  - "Enable and use the ROS 2 Bridge to connect Isaac Sim to ROS"
  - "Write a Python script to spawn an object in the simulation"
---

# Week 6: Introduction to Isaac Sim

Welcome to the world of high-fidelity robotics simulation! **NVIDIA Isaac Sim** is a powerful robotics simulation platform built on **NVIDIA Omniverse**. It allows for the development, testing, and training of AI-based robots in a photorealistic, physically-accurate virtual environment.

## The Omniverse Platform

Isaac Sim is an application built on Omniverse, a platform designed for 3D workflows and virtual world simulation. It uses **USD (Universal Scene Description)** as its native file format, which allows for seamless collaboration and asset interchange between different 3D applications.

## The ROS 2 Bridge

One of the most powerful features of Isaac Sim is its built-in **ROS 2 Bridge**. This allows Isaac Sim to connect directly to a ROS 2 workspace, enabling bi-directional communication. You can:
-   Publish sensor data (cameras, LiDAR) from the simulation to ROS 2 topics.
-   Subscribe to ROS 2 topics to control robots in the simulation.
-   Call and provide ROS 2 services and actions.

## Code Example: Spawning a Cube

This example uses the Isaac Sim Python API to create a simple script that adds a cube to the simulation world.

```python
# spawn_cube.py
from omni.isaac.core import World
from omni.isaac.core.objects import cuboid

# Create a new world
my_world = World()

# Add a cuboid to the stage at position (0, 0, 1.0)
# The default world units are in meters
my_world.scene.add(
    cuboid.VisualCuboid(
        prim_path="/new_cube", # The path in the USD stage
        name="my_visual_cube",
        position=[0, 0, 1.0],
        scale=[0.5, 0.5, 0.5],
        color=[1.0, 0, 0], # Red
    )
)

# Reset the world to ensure the cube is loaded
my_world.reset()

# Keep the script running to see the cube in the simulation
# In a real application, you would add more logic here.
while True:
    my_world.step(render=True)

```

### How to Run

1.  Open Isaac Sim.
2.  Go to `Window` -> `Script Editor`.
3.  Copy and paste the code into the Script Editor window.
4.  Press the "Run" button.
5.  You will see a red cube appear in the center of the simulation environment. This demonstrates direct, programmatic control over the simulation environment (the "Stage").
