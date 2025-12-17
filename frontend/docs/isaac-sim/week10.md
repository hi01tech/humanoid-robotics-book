---
id: week10
title: 'Module 3: Synthetic Data Generation'
sidebar_label: 'Week 10: Synthetic Data'
---

## Week 10: Generating Synthetic Data for Perception

One of the most powerful features of Isaac Sim is its ability to generate large, high-quality, and automatically labeled datasets for training perception models. In robotics, collecting and labeling real-world data is often expensive, time-consuming, and sometimes dangerous. Synthetic data provides a solution to bootstrap and augment real-world datasets.

### What is Synthetic Data?

Synthetic data is data that is generated artificially rather than being collected from the real world. In the context of Isaac Sim, this typically refers to:
*   **Photorealistic Images:** Rendered images from cameras within the simulation.
*   **Labels and Annotations:** Pixel-perfect annotations that are generated automatically alongside the images. These can include:
    *   **Bounding Boxes (2D and 3D):** Boxes drawn around objects of interest.
    *   **Semantic Segmentation:** A label for every pixel in the image, indicating what type of object it belongs to (e.g., "robot," "table," "floor").
    *   **Instance Segmentation:** Similar to semantic segmentation, but distinguishes between different instances of the same object type.
    *   **Depth Images:** An image where each pixel's value represents the distance from the camera to that point in the scene.

Because this data is generated from the simulation ground truth, the labels are perfectly accurate and require no manual effort.

### Domain Randomization

A key challenge when using synthetic data is the "domain gap" - the difference between the simulated and real worlds. A model trained purely on non-randomized synthetic data may not perform well on real-world images. To overcome this, we use **Domain Randomization**.

Domain Randomization involves randomly changing aspects of the simulation environment during data generation. This forces the model to learn the essential features of the objects it's trying to detect, rather than memorizing the specific details of the simulated environment.

In Isaac Sim, you can randomize:
*   **Textures and Materials:** The appearance of objects, walls, and floors.
*   **Lighting:** The position, orientation, color, and intensity of lights.
*   **Object Pose:** The position and orientation of objects in the scene.
*   **Camera Pose:** The position and orientation of the camera.

By training on a dataset with wide domain randomization, the real world can appear to the model as just another variation of the simulation.

### The Replicator Workflow in Isaac Sim

Isaac Sim includes a powerful tool called the **Replicator** for setting up and running synthetic data generation pipelines. The Replicator works as a graph, much like the Action Graph, allowing you to define the randomization and data output process.

Here is a conceptual Python script using the Replicator API to generate a dataset of images with bounding box labels:

```python
import omni.replicator.core as rep

# Define the paths to your 3D models (USD files)
# These could be objects you want to learn to detect
OBJECT_ASSET_PATHS = ["/path/to/object1.usd", "/path/to/object2.usd"]

# --- Define Randomizers ---

# 1. Randomizer for creating the objects themselves
def create_random_objects():
    # Create a layer to hold the objects
    with rep.layer("objects"):
        # For each object, add it to the scene at a random pose
        rep.create.from_usd(
            rep.distribution.choice(OBJECT_ASSET_PATHS),
            count=5 # Create 5 objects per frame
        )
    return rep.get.prims(from_layer="objects")

# 2. Randomizer for the material/look of the objects
def randomize_materials(prims):
    # For the given prims, apply a random material
    with prims:
        rep.modify.material(
            rep.distribution.choice(
                [rep.get.material("/path/to/material1.mdl"), rep.get.material("/path/to/material2.mdl")]
            )
        )
    return prims

# 3. Randomizer for the pose of the objects
def randomize_pose(prims):
    with prims:
        rep.modify.pose(
            position=rep.distribution.uniform((-100, 0, 0), (100, 0, 100)), # Random x and z position
            rotation=rep.distribution.uniform((0, -180, 0), (0, 180, 0)) # Random yaw
        )
    return prims

# --- Register Randomizers with the Replicator Trigger ---

# `rep.trigger.on_frame()` causes this graph to be re-evaluated every frame
with rep.trigger.on_frame():
    # Chain the randomizers together
    prims = create_random_objects()
    prims = randomize_materials(prims)
    prims = randomize_pose(prims)
    
# --- Define the Output (Annotator) ---

# 1. Create a camera
camera = rep.create.camera()

# 2. Create an annotator to generate the data
annotator = rep.AnnotatorRegistry.get_annotator("bounding_box_2d_tight")
annotator.attach([camera])

# 3. Tell the Replicator what to output
# The output will be saved in a folder structure specified by the writer
writer = rep.WriterRegistry.get("BasicWriter")
writer.initialize(output_dir="_output", rgb=True, bounding_box_2d_tight=True)
writer.attach([camera])

# To run this, you would typically use the `omni.replicator.core.orchestrator.run()` function
```

This script sets up a pipeline that, on every frame, creates 5 random objects, assigns them random materials, and places them at random positions. It then captures an image from a camera and saves both the image and the corresponding 2D bounding box data to a directory. This output can then be directly used to train a model with a framework like PyTorch or TensorFlow.

Synthetic data generation is a transformative technology for robotics, and Isaac Sim provides a state-of-the-art toolset to leverage it.