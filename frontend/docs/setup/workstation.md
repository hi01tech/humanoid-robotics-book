---
id: setup-workstation
title: Workstation Setup Guide
sidebar_label: Workstation Setup
estimated_time: 2
week: 1
module: Foundations
prerequisites: []
learning_objectives:
  - "Install Ubuntu 22.04 LTS"
  - "Install NVIDIA drivers for your RTX GPU"
  - "Install ROS 2 Iron Irwini"
  - "Install NVIDIA Isaac Sim"
---

# Digital Twin Workstation Setup Guide

This guide will walk you through setting up your personal computer as a development workstation for the "Physical AI & Humanoid Robotics" textbook. This setup is ideal for users with a dedicated NVIDIA RTX GPU.

## System Requirements

-   **Operating System**: Ubuntu 22.04 LTS
-   **GPU**: NVIDIA RTX Series (e.g., RTX 30-series, 40-series)
-   **Storage**: At least 100GB of free space
-   **Memory (RAM)**: At least 16GB, 32GB recommended

## Step 1: Install Ubuntu 22.04 LTS

If you do not already have Ubuntu 22.04 installed, please download it from the [official Ubuntu website](https://ubuntu.com/download/desktop) and create a bootable USB drive to install it.

## Step 2: Install NVIDIA Drivers

After installing Ubuntu, you need to install the appropriate NVIDIA drivers for your GPU.

1.  Open the "Software & Updates" application.
2.  Go to the "Additional Drivers" tab.
3.  Select the latest proprietary NVIDIA driver (e.g., `nvidia-driver-535`).
4.  Click "Apply Changes" and restart your computer when prompted.

You can verify the installation by opening a terminal and running:
```bash
nvidia-smi
```
This command should display information about your NVIDIA GPU.

## Step 3: Install ROS 2 Iron Irwini

Follow the official ROS 2 documentation to install ROS 2 Iron Irwini for Ubuntu. You can find the instructions at the [ROS 2 Iron Installation Guide](https://docs.ros.org/en/iron/Installation/Ubuntu-Install-Debians.html).

Make sure to source the ROS 2 setup file in your `.bashrc` to have it available in all your terminals:
```bash
echo "source /opt/ros/iron/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

## Step 4: Install NVIDIA Isaac Sim

1.  Download and install the Omniverse Launcher from the [NVIDIA Isaac Sim website](https://developer.nvidia.com/isaac-sim).
2.  Open the Omniverse Launcher, go to the "Exchange" tab, and find "Isaac Sim".
3.  Install the latest version compatible with the course (2023.1).
4.  Once installed, you can launch Isaac Sim from the "Library" tab.

Your workstation is now set up and ready for the course!
