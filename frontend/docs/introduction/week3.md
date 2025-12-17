---
id: foundations-week3
title: "Week 3: State Estimation & Introduction to ROS"
sidebar_label: "Week 3: State Estimation"
estimated_time: 4
week: 3
module: Foundations
prerequisites: ["foundations-week2"]
learning_objectives:
  - "Define state estimation and its importance in robotics"
  - "Understand the concept of SLAM (Simultaneous Localization and Mapping)"
  - "Explain the core concepts of ROS 2 (nodes, topics, services)"
  - 'Build a simple "Hello, World" program in ROS 2'
---

# Week 3: State Estimation & Introduction to ROS

In our final foundational week, we tackle one of the most critical problems in mobile robotics: **state estimation**. A robot must know where it is to perform any useful task. We will explore how robots build maps and locate themselves within them.

We will also have our first introduction to the **Robot Operating System (ROS)**, the software framework that will be the backbone for all our projects.

## Topics Covered

-   Probabilistic robotics and Bayes filters.
-   Overview of SLAM algorithms.
-   Introduction to the ROS 2 architecture.
-   Creating your first ROS 2 package and node.

# What is ROS 2?



The **Robot Operating System 2 (ROS 2)** is an open-source, flexible framework for writing robot software. It is not an operating system itself, but rather a **set of software libraries, tools, and conventions** that simplify the complex task of building robot applications.





Key features of ROS 2 include:

* **Publish/Subscribe communication model** for message passing.

* **Multi-platform Support:** Native compatibility with Linux, Windows, and macOS.

* **Real-time support** which is critical for industrial and complex robotic systems.

* **Improved security and quality of service (QoS)** Uses the industry-standard **DDS (Data Distribution Service)** as its communication middleware for improved security and performance. compared to the original ROS 1.



### Core Concepts of ROS 2



ROS 2 architecture is built around several key communication primitives:



1.  **Nodes:** These are individual processes (executables) that perform specific tasks, like reading sensor data, calculating path planning, or controlling motors. Every operational unit in ROS 2 is a Node.

2.  **Topics:** This is the asynchronous, publish-subscribe system used for one-way, continuous streaming of data (like camera images or odometry). A node publishes data to a Topic, and other nodes subscribe to it. 

3.  **Services:** This is a synchronous system used for a request/response pattern (like asking a robot to turn 90 degrees and waiting for a confirmation).

*(Content to be developed)*
