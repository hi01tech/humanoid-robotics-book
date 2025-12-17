---
id: week8
title: 'Module 3: Introduction to Isaac Sim'
sidebar_label: 'Week 8: Isaac Sim Basics'
---

## Week 8: Getting Started with NVIDIA Isaac Sim

Welcome to Module 3! Now we dive into the core of our digital twin strategy: **NVIDIA Isaac Sim**. Isaac Sim is a scalable robotics simulation application and synthetic data generation tool. It's built on the NVIDIA Omniverse™ platform and provides a photorealistic, physically accurate environment for developing, testing, and training AI-based robots.

### Why Isaac Sim?

For modern robotics, especially when dealing with complex humanoids and AI-driven behaviors, Isaac Sim offers several key advantages over other simulators:
*   **Physics-based Simulation:** It leverages the NVIDIA PhysX 5 engine, providing high-fidelity physics simulation that is crucial for realistic robot behavior.
*   **Photorealistic Rendering:** Built on Omniverse, Isaac Sim can produce stunningly realistic visuals using real-time ray tracing. This is essential for generating synthetic data to train computer vision models.
*   **ROS/ROS 2 Integration:** It has first-class support for ROS 2, allowing for seamless communication between your ROS 2 nodes and the simulated world. We've seen this in action in Module 2.
*   **Python Scripting:** The entire simulator can be controlled and customized through Python scripts, enabling automation, custom workflows, and integration with other tools.

### The Isaac Sim Interface

When you first launch Isaac Sim, you are greeted with the Omniverse interface. The key windows to familiarize yourself with are:
*   **Viewport:** The main window where you see and interact with your 3D scene.
*   **Stage:** A hierarchical representation of all the elements (called "prims") in your scene. This is similar to a file system view of your virtual world.
*   **Property Panel:** Displays all the properties of the currently selected prim. Here you can change a prim's position, orientation, material, physics properties, and more.
*   **Content Browser:** Your file browser for finding assets like 3D models (USD, URDF), materials, and environments.

### Creating Your First Scene

Let's walk through the process of creating a very simple scene in Isaac Sim.

1.  **Create a Ground Plane:** Go to `Create > Physics > Ground Plane`. This adds a flat surface for your robot and objects to rest on.
2.  **Add a Light:** A scene needs light to be visible. Go to `Create > Light > Distant Light`. You can position and rotate the light to change the direction of the "sun."
3.  **Add a Primitive Shape:** Let's add a simple cube. Go to `Create > Shape > Cube`. You can use the manipulation gizmos in the Viewport or the Property Panel to move, rotate, and scale the cube.
4.  **Add Physics to the Cube:** By default, the cube is just a visual object. To make it interact with the world, we need to add physics properties.
    *   Select the cube prim in the Stage.
    *   In the Property Panel, click `+ Add > Physics > Rigid Body`. This will give the cube properties like mass and velocity.
    *   Also add `+ Add > Physics > Collision`. Now the cube will collide with other physics-enabled objects, like the ground plane.
5.  **Press Play!** At the top of the interface, you'll see a "Play" button. Pressing it starts the physics simulation. If you moved your cube up in the air, you will see it fall and land on the ground plane.

### Scripting with Python

The true power of Isaac Sim comes from scripting. You can open a scripting window via `Window > Script Editor`. Here, you can write and execute Python code to control every aspect of the simulation.

Here is a simple script that finds our cube and moves it.

```python
import omni.usd
from pxr import Gf

# Get the stage
stage = omni.usd.get_context().get_stage()

# Get the prim for our cube
# Note: The actual path might be different depending on how you created it.
# Check the Stage window for the correct path.
cube_prim = stage.GetPrimAtPath("/World/Cube")

# Check if the prim exists
if cube_prim:
    # Get the "xformable" interface, which allows us to move, rotate, and scale the prim
    xformable = omni.usd.get_xform_cache().Get(cube_prim)
    
    # Get the current position
    current_translation = xformable.GetLocalTransform()[3]
    print(f"Current Cube Position: {current_translation}")

    # Set a new position
    new_position = Gf.Vec3d(100.0, 0.0, 50.0) # Move it to (100, 0, 50) in cm
    xformable.SetTranslate(new_position)
    print(f"New Cube Position: {xformable.GetLocalTransform()[3]}")
else:
    print("Could not find a prim at path '/World/Cube'. Please check the path in the Stage window.")

```

This is just the beginning. Next week, we will focus on importing our robot's URDF into Isaac Sim and starting it with the ROS 2 bridge to connect it to the ROS 2 ecosystem we've been building.