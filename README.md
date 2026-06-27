# Autonomous Mobile Robot (AMR) using ROS2 Jazzy

## Overview

This repository contains the complete software architecture for an Autonomous Mobile Robot (AMR) developed using ROS2 Jazzy.

The system is designed to perform autonomous lane following and obstacle avoidance in a structured environment. It integrates computer vision, sensor fusion, real-time control, and navigation algorithms to enable reliable autonomous operation on a physical mecanum-wheel robot platform.

The project was developed as part of the Design of Autonomous Systems course and includes both simulation and real-world hardware implementation.

---

## Key Features

* Real-time lane detection using OpenCV
* Autonomous lane following
* Obstacle detection and avoidance
* Dynamic lane switching
* IMU-based orientation estimation
* Mecanum wheel motion control
* Modular ROS2 package architecture
* Web-based monitoring interface
* Physical robot deployment on Raspberry Pi 5

---

## System Architecture

The software is organized into independent ROS2 packages:

| Package                  | Function                                   |
| ------------------------ | ------------------------------------------ |
| `camera_pkg`             | Camera acquisition and lane detection      |
| `camera_interfaces`      | Custom ROS2 interfaces and messages        |
| `imu_pkg`                | IMU integration and orientation estimation |
| `mecanum_controller_pkg` | Robot kinematics and motion control        |
| `my_auto_sensors`        | Sensor integration and obstacle detection  |
| `navigation_pkg`         | Navigation logic and lane switching        |
| `robot_gui`              | Web dashboard and visualization            |

---

## Hardware Platform

### Processing Unit

* Raspberry Pi 5

### Sensors

* USB Camera
* IMU
* Distance Sensors

### Drive System

* Four-wheel Mecanum Mobile Platform

---

## Navigation Tasks

### Lane Following

The robot detects lane boundaries using computer vision and continuously computes the lane center error to maintain stable trajectory tracking.

### Obstacle Avoidance

The robot detects obstacles along the track and performs autonomous lane-switching maneuvers while maintaining safe navigation and minimizing deviation from the planned path.

---

## Technologies Used

* ROS2 Jazzy
* Python
* OpenCV
* Ubuntu 24.04
* HTML / CSS / JavaScript
* Raspberry Pi 5

---

## Repository Structure

```text
.
├── camera_interfaces
├── camera_pkg
├── imu_pkg
├── mecanum_controller_pkg
├── my_auto_sensors
├── navigation_pkg
└── robot_gui
```

---

## Results

The system was successfully deployed on a physical robot platform and demonstrated:

* Reliable lane tracking
* Real-time obstacle detection
* Autonomous lane switching
* Stable navigation performance
* Successful completion of the required navigation tasks

---

## Demonstration

### Autonomous Robot

<p align="center">
  Add robot image here
</p>

### Lane Detection

<p align="center">
  Add lane detection image or GIF here
</p>

### Obstacle Avoidance

<p align="center">
  Add obstacle avoidance video or GIF here
</p>

---

## Build

```bash
colcon build
source install/setup.bash
```

---

## Author

**Nour Eldin** And **Mark George**

Mechatronics Engineering Student

Robotics • ROS2 • Embedded Systems • Computer Vision • Autonomous Systems
