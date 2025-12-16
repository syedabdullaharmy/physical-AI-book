---
sidebar_position: 19
---

# Lab Setup Instructions

Step-by-step setup guide for the Physical AI development environment.

## 1. Install Ubuntu 22.04 LTS

Download from [ubuntu.com](https://ubuntu.com/download/desktop)

## 2. Install ROS 2 Humble

```bash
sudo apt update
sudo apt install ros-humble-desktop python3-colcon-common-extensions
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```

## 3. Install NVIDIA Drivers

```bash
sudo apt install nvidia-driver-535
sudo reboot
```

## 4. Verify Installation

```bash
ros2 --version
nvidia-smi
```

Complete setup guide available in course materials.
