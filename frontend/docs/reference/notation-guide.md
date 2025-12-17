--- 
id: notation-guide
title: "Notation Guide"
sidebar_label: "Notation Guide"
estimated_time: 0.5
week: 1 # Accessible throughout
module: "Reference"
prerequisites: []
learning_objectives:
  - "Understand the common mathematical and symbolic notations used in the textbook"
---

# Notation Guide

This guide outlines the mathematical and symbolic notations used consistently throughout the "Physical AI & Humanoid Robotics" textbook.

## Common Symbols

| Symbol | Description |
|---|---|
| $ \mathbf{q} $ | Joint angles/positions vector |
| $ \mathbf{\dot{q}} $ | Joint velocities vector |
| $ \mathbf{\tau} $ | Joint torques vector |
| $ \mathbf{x} $ | End-effector position vector (Cartesian) |
| $ \mathbf{R} $ | Rotation matrix (e.g., $ \mathbf{R}_{B}^{A} $ denotes rotation from frame B to A) |
| $ \mathbf{p} $ | Position vector |
| $ \mathbf{T} $ | Homogeneous Transformation Matrix |
| $\Sigma$ | Summation |
| $\prod$ | Product |
| $ \nabla $ | Gradient operator |
| $ \partial $ | Partial derivative |
| $ \mathbb{R}^n $ | n-dimensional real vector space |
| $ \mathcal{L} $ | Lagrangian |
| $ \mathcal{H} $ | Hamiltonian |

## Coordinate Frames

Throughout the textbook, we will use a consistent notation for coordinate frames:

-   Superscripts denote the reference frame, and subscripts denote the frame being described.
-   For example, $ \mathbf{p}^A_B $ represents the position of origin of frame B as seen from frame A.
-   $ \mathbf{R}^{A}_{B} $ represents the rotation matrix that transforms vectors from frame B to frame A.

## Vector and Matrix Notation

-   **Bold lowercase letters** (e.g., $ \mathbf{v} $) denote vectors.
-   **Bold uppercase letters** (e.g., $ \mathbf{M} $) denote matrices.
-   Scalars are denoted by regular lowercase letters (e.g., $ s $).

## ROS 2 Specifics

| Notation | Description |
|---|---|
| `/topic_name` | A ROS 2 topic |
| `/service_name` | A ROS 2 service |
| `<package_name>` | A ROS 2 package name |
| `[node_name]` | A ROS 2 node name |
| `msg/MsgType` | A ROS 2 message type |
| `srv/SrvType` | A ROS 2 service type |
| `action/ActionType` | A ROS 2 action type |

*(Content to be expanded with more specific robotics notations as chapters are developed)*