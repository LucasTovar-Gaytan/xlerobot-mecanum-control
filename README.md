# XLeRobot Mecanum Control & Teleoperation

This repository contains custom control scripts for driving a 4-wheel mecanum robotic base using the Hugging Face [LeRobot](https://github.com/huggingface/lerobot) framework. It includes both manual keyboard teleoperation and advanced 1:1 positional tracking designed for VR headset integration.

## 📁 Files Included

### 1. `keyboard_drive.py`
A manual teleoperation script that allows you to drive the mecanum base using standard keyboard inputs. It translates basic directional inputs into the appropriate forward, reverse, strafing, and rotational wheel commands for the 4-wheel chassis.

### 2. `mecanum_base.py`
An advanced physics and inverse kinematics controller for **1:1 Spatial Tracking**. 
* **How it works:** It is designed to take live spatial coordinates (X position, Z position, and Yaw rotation) from a VR headset (like a Meta Quest) and mathematically translate your physical walking movements into robot wheel velocities in real-time.
* **Kinematics:** Implements full mecanum inverse kinematics using the robot's specific physical dimensions (axle length, width, and wheel radius) to ensure accurate directional strafing and rotation.
* **Safety limits:** Includes built-in safety scaling to ensure mathematical outputs never exceed the maximum safe operating speed of the STS3215 servos.

## ⚙️ Hardware Setup
* **Chassis:** 4-Wheel Mecanum Base
* **Motors:** STS3215 Bus Servos 
  * Front Left: ID 9
  * Front Right: ID 10
  * Back Left: ID 11
  * Back Right: ID 12
* **Framework:** Hugging Face LeRobot (`XLerobot` interface)

## 🚀 How to Run

Ensure your robot is powered on and connected, and that your Python environment has the LeRobot framework active.

To run the positional tracking loop:
```bash
python3 mecanum_base.py
