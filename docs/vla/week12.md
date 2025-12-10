---
id: vla-week12
title: "Week 12: Action Tokenization and Policy Learning"
sidebar_label: "Week 12: Action Tokenization"
estimated_time: 4
week: 12
module: "VLA"
prerequisites:
  - "vla-week11"
learning_objectives:
  - "Understand how continuous robot actions are discretized into tokens"
  - "Explore various methods of action tokenization"
  - "Grasp the concepts of policy learning in the context of VLA models"
  - "Discuss the role of imitation learning and reinforcement learning in VLA training"
---

# Week 12: Action Tokenization and Policy Learning

This week, we will dive deeper into the mechanics of how VLA models generate robot movements. A key challenge in connecting high-level language commands and visual observations to low-level robot control is **action tokenization**.

## Action Tokenization

Robot actions are typically continuous (e.g., joint angles, end-effector velocities). For VLA models that leverage transformer architectures (originally designed for discrete language tokens), these continuous actions must be converted into a discrete, tokenized format. We will explore different strategies for achieving this, including discretizing continuous spaces and predicting action primitives.

## Policy Learning

We will then investigate how VLA models learn to map observations and instructions to these action tokens. This involves various techniques from machine learning, particularly:

-   **Imitation Learning**: Training models by demonstrating desired behaviors.
-   **Reinforcement Learning**: Learning through trial and error, guided by reward signals.

*(Content to be developed - will leverage NVIDIA GR00T and OpenVLA resources as researched)*
