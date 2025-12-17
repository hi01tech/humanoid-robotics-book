---
id: setup-edge-kit
title: Edge Kit Setup Guide (Jetson Orin Nano)
sidebar_label: Edge Kit Setup
estimated_time: 2.5
week: 1
module: Foundations
prerequisites: []
learning_objectives:
  - "Flash the Jetson Orin Nano with the latest JetPack"
  - "Install ROS 2 Iron Irwini"
  - "Configure the Jetson for robotics applications"
---

# Physical AI Edge Kit Setup Guide

This guide details how to set up the NVIDIA Jetson Orin Nano Developer Kit, which serves as the physical AI edge computing device for this course.

## System Requirements

-   NVIDIA Jetson Orin Nano Developer Kit
-   microSD card (at least 64GB, U3 recommended)
-   USB-C power supply (5V, 3A minimum)
-   Host computer (for flashing the microSD card)

## Step 1: Flash the Jetson Orin Nano

1.  Download the latest Jetson Orin Nano Developer Kit SD Card Image from the [NVIDIA Jetson website](https://developer.nvidia.com/jetson-orin-nano-devkit-sd-card-image).
2.  Use a tool like [balenaEtcher](https://www.balena.io/etcher/) to flash the downloaded image onto your microSD card.
3.  Insert the microSD card into the Jetson Orin Nano, connect your peripherals (keyboard, mouse, monitor), and power it on.
4.  Complete the initial Ubuntu setup process.

## Step 2: Install ROS 2 Iron Irwini

The process for installing ROS 2 on the Jetson is similar to a desktop, but you'll be using packages built for the ARM architecture.

Follow the official ROS 2 documentation to install ROS 2 Iron Irwini. You can find the instructions at the [ROS 2 Iron Installation Guide](https://docs.ros.org/en/iron/Installation/Ubuntu-Install-Debians.html).

## Step 3: Configure the System

For robotics applications, it's useful to maximize the performance of the Jetson device.

```bash
# Set power mode to MAXN
sudo nvpmodel -m 0
```

Your Jetson Orin Nano is now configured and ready to be used for the labs in this course.
