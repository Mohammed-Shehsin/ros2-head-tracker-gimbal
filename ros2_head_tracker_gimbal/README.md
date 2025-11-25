# Head Tracker Gimbal – ROS2 URDF Project

This repository contains a ROS2 workspace with a description package for a **2-DOF head-tracker gimbal** driven by two micro servos (yaw + pitch).  
The goal is to link a real physical gimbal (MG90S-based pan/tilt bracket) with a ROS2 model for face tracking and other experiments.

---

## 1. Project Overview

- **Hardware**
  - 2-axis aluminium pan/tilt gimbal for 9g servos
  - 2× MG90S micro servos (yaw + pitch)
  - Base plate approx. 60×40 mm
  - Camera/plate mounted on the pitch axis

- **Software**
  - Ubuntu 22.04 (assumption, adjust if different)
  - ROS2 (e.g. Humble – update if you use another distro)
  - RViz2 for visualization
  - `gimbal_description` ROS2 package with URDF/Xacro model

Current state: simple geometric model using boxes (base, servo bodies, column, camera plate) that approximates the real gimbal dimensions and joint axes.

---

## 2. Repository Structure

```text
ros2_head_tracker_gimbal/
├── src/
│   └── gimbal_description/
│       ├── urdf/
│       │   └── head_tracker_gimbal.urdf.xacro
│       ├── launch/
│       │   └── display.launch.py
│       ├── rviz/
│       │   └── display.rviz
│       ├── package.xml
│       └── setup.py / setup.cfg
├── .gitignore
└── README.md
