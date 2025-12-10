---
id: glossary
title: "Glossary of Terms"
sidebar_label: "Glossary"
estimated_time: 1
week: 1 # Accessible throughout
module: "Reference"
prerequisites: []
learning_objectives:
  - "Provide quick definitions for key terms used in Physical AI and Humanoid Robotics"
---

# Glossary of Terms

This glossary provides definitions for key terms used throughout the "Physical AI & Humanoid Robotics" textbook.

*(Content to be developed - will be populated with 100+ robotics terms)*

---

### **A**

**Action Tokenization**: The process of converting continuous robot actions into a discrete, tokenized format suitable for processing by large language models.

**Actuator**: A component of a machine that is responsible for moving and controlling a mechanism or system. It takes energy, usually created by air, electric current, or liquid, and converts that into motion.

**Artificial Intelligence (AI)**: The simulation of human intelligence processes by machines, especially computer systems. These processes include learning, reasoning, and self-correction.

---

### **C**

**Capstone Project**: A culminating academic experience that enables students to integrate knowledge and skills acquired in their courses, apply them to a practical problem, and demonstrate mastery of their discipline.

**Coordinate Frame**: A system used to describe the position and orientation of objects in space relative to a fixed origin and axes. In robotics, multiple coordinate frames are used (e.g., world frame, robot base frame, end-effector frame).

**`ros2_control`**: A set of packages that provide a generic control architecture for ROS 2 robots, enabling standardized control of robot hardware.

---

### **D**

**Digital Twin**: A virtual model designed to accurately reflect a physical object, process, or system. In robotics, it refers to a high-fidelity simulation environment (like Isaac Sim) that mirrors a physical robot.

**Docusaurus**: A static site generator that helps you build optimized websites quickly. It's particularly popular for documentation websites.

**Degrees of Freedom (DoF)**: The number of independent parameters that define the configuration of a mechanical system. For a robot arm, this often refers to the number of joints that can move independently.

---

### **I**

**Isaac ROS**: A collection of GPU-accelerated packages for ROS 2 that provide high-performance solutions for common robotics tasks like perception, navigation, and manipulation, optimized for NVIDIA hardware.

**Isaac Sim**: An extensible, scalable robotics simulation platform built on NVIDIA Omniverse, used for developing, testing, and training AI-based robots.

**Inverse Kinematics (IK)**: The calculation of the joint parameters (angles or displacements) of a robot manipulator required to achieve a desired position and orientation of the end-effector.

---

### **K**

**Kalman Filter**: An algorithm that uses a series of measurements observed over time, containing statistical noise and other inaccuracies, to produce estimates of unknown variables that tend to be more precise than those based on a single measurement alone.

**Kinematics**: The branch of mechanics concerned with the motion of objects without reference to the forces that cause the motion. In robotics, it describes the relationship between the joint angles of a robot and the position/orientation of its end-effector.

---

### **L**

**Launch File**: In ROS 2, an XML or Python file that describes how to run a set of nodes and other processes, making it easy to start complex robotic systems.

**Link**: In a robot's kinematic chain (URDF), a rigid body that is connected to other links by joints.

---

### **N**

**Natural Language Processing (NLP)**: A field of artificial intelligence that gives computers the ability to understand, process, and generate human language.

**NVIDIA Omniverse**: A platform for connecting and building custom 3D pipelines based on Universal Scene Description (USD). Isaac Sim is built on Omniverse.

---

### **P**

**Parameter (ROS 2)**: A configurable value associated with a ROS 2 node, allowing runtime modification of node behavior without recompiling code.

**Physical AI**: A branch of artificial intelligence focused on intelligent systems that interact with the physical world, often embodied in robots.

**PID Controller**: A proportional-integral-derivative controller is a control loop mechanism employing feedback that is widely used in industrial control systems and a variety of other applications requiring continuously modulated control.

**Policy Learning**: In reinforcement learning, the process of learning a policy (a mapping from states to actions) that maximizes a reward signal over time.

---

### **R**

**Real-Time Control**: Control systems that respond to inputs within a guaranteed time frame, crucial for safe and effective robot operation.

**Robot Operating System (ROS)**: An open-source, meta-operating system for robots. It provides services like hardware abstraction, low-level device control, implementation of commonly used functionalities, message-passing between processes, and package management.

---

### **S**

**Sensor Fusion**: The process of combining data from multiple sensors to produce a more complete, accurate, or reliable understanding of an environment or system than could be achieved by using individual sensors alone.

**Simultaneous Localization and Mapping (SLAM)**: The computational problem of constructing or updating a map of an unknown environment while simultaneously keeping track of an agent's location within it.

**State Estimation**: The process of using sensor measurements to infer the current state (e.g., position, velocity, orientation) of a system.

---

### **T**

**`tf2`**: The ROS 2 library for managing coordinate transforms. It allows you to keep track of multiple coordinate frames and transform points, vectors, and other entities between them.

**TensorRT**: An SDK for high-performance deep learning inference. It includes a deep learning inference optimizer and runtime that delivers low latency and high throughput for deep learning inference applications.

**Topic (ROS 2)**: An anonymous publish/subscribe communication mechanism in ROS 2. Nodes publish messages to topics, and other nodes subscribe to topics to receive those messages.

---

### **U**

**Unified Robot Description Format (URDF)**: An XML file format in ROS used to describe a robot's physical structure, including its links and joints.

**Universal Scene Description (USD)**: An open-source 3D scene description technology developed by Pixar for content creation and interchange. It's the native format for NVIDIA Omniverse.

---

### **V**

**Vision-Language-Action (VLA) Models**: AI models that integrate visual perception, natural language understanding, and robotic action generation to enable robots to perform tasks based on high-level commands and environmental context.

**Vision-Language Models (VLM)**: AI models that understand and generate content based on both visual (images/video) and textual data. VLMs are a precursor to VLA models.
