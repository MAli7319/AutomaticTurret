# 🛡️ Automatic Turret: Autonomous Target Acquisition System

Welcome to the **Automatic Turret** project—a ROS 2-based system that enables an armored vehicle to autonomously detect and track targets without human intervention. Leveraging the power of YOLOv8 for object detection and related camera parameters, this system calculates the target's position relative to the camera and commands the turret to align accordingly.

---

## 📌 Features

- **Real-time Object Detection**: Utilizes [YOLOv8](https://github.com/ultralytics/ultralytics) for efficient and accurate target detection.
- **Calculating the Position**: Employs some of the intrinsic camera parameters to determine the target's position with respect to the camera.
- **Autonomous Turret Control**: Calculates the necessary joint angles and commands the turret to align with the detected target.
- **ROS 2 Integration**: Built upon the ROS 2 framework for modularity and scalability.
- **Simulation Support**: Compatible with simulation environments for testing and development.

---

## 🎯 System Workflow

- **Image Acquisition**: The system captures images from the onboard camera.
- **Target Detection**: YOLOv8 processes the images to detect potential targets.
- **Position Calculation**: Using camera intrinsics, the system calculates the target's position relative to the vehicle.
- **Turret Alignment**: The calculated position is translated into joint angles, and commands are sent to align the turret with the target.


## YouTube Overview

[![IMAGE ALT TEXT HERE](https://img.youtube.com/vi/f3K92w8VBuU/0.jpg)](https://www.youtube.com/watch?v=f3K92w8VBuU)
