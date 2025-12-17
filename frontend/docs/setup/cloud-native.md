---
id: setup-cloud-native
title: Cloud-Native Setup Guide (AWS/Azure)
sidebar_label: Cloud-Native Setup
estimated_time: 1.5
week: 1
module: Foundations
prerequisites: []
learning_objectives:
  - "Launch a GPU-enabled virtual machine on AWS or Azure"
  - "Connect to the virtual machine using a remote desktop client"
  - "Understand the basics of using Isaac Sim in the cloud"
---

# Cloud-Native Setup Guide

This guide explains how to set up a cloud-based development environment using either AWS or Azure. This is a great option if you do not have a local machine with a powerful NVIDIA GPU.

**Note:** While the NVIDIA software is free, using cloud services will incur costs based on the provider's pricing for GPU instances.

## Option 1: AWS Setup

1.  **Prerequisites**: An AWS account and the [NICE DCV client](https://download.nice-dcv.com/) installed on your local machine.
2.  **Launch Instance**:
    -   Navigate to the AWS Marketplace and search for the **"NVIDIA Isaac Sim™ Development Workstation (Linux)"** AMI.
    -   Select the AMI and choose the `g6e.2xlarge` instance type, which includes an NVIDIA L40S GPU.
    -   Configure and launch the instance, ensuring you create and associate a key pair that you have access to.
3.  **Connect**:
    -   Follow the instructions provided by the AMI to connect to the instance using the NICE DCV client and the key pair you created.

## Option 2: Azure Setup

1.  **Prerequisites**: A Microsoft Azure account.
2.  **Launch Instance**:
    -   Navigate to the Azure Marketplace and search for the **"NVIDIA Isaac Sim Development Workstation VMI"**.
    -   Select the image and create a new Virtual Machine.
    -   Choose the `Standard_NV36ads_A10_v5` instance size, which includes an NVIDIA A10 GPU.
    -   Configure and launch the VM.
3.  **Connect**:
    -   Use a remote desktop client (like Remote Desktop for Windows or Cendio's ThinLinc for Linux) to connect to the graphical desktop of the VM.

Your cloud-based workstation is now ready. You can launch Isaac Sim from the desktop environment.
