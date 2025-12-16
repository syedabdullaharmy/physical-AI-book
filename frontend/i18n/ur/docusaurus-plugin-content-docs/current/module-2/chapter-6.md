---
sidebar_position: 6
---

# Chapter 6: Gazebo Simulation Environment

## Introduction

**Gazebo** is the industry-standard simulator for robotics. It provides realistic physics, sensors, and environments for testing robots before deploying to hardware.

:::tip Learning Objectives
- Set up Gazebo with ROS 2
- Create simulation worlds
- Simulate physics and sensors
- Spawn and control robots
- Integrate with ROS 2 ecosystem
:::

## Gazebo Architecture

```
┌─────────────────────────────────────┐
│      Gazebo Server (gzserver)       │
│  - Physics engine                   │
│  - Sensor simulation                │
│  - Plugin system                    │
└─────────────────────────────────────┘
             ↕
┌─────────────────────────────────────┐
│      Gazebo Client (gzclient)       │
│  - 3D visualization                 │
│  - User interface                   │
└─────────────────────────────────────┘
             ↕
┌─────────────────────────────────────┐
│         ROS 2 Bridge                │
│  - Topic publishing                 │
│  - Service calls                    │
│  - Transform broadcasting           │
└─────────────────────────────────────┘
```

## Installing Gazebo

```bash
# Install Gazebo for ROS 2 Humble
sudo apt install ros-humble-gazebo-ros-pkgs ros-humble-gazebo-ros2-control

# Verify installation
gazebo --version
```

## Creating a World

**File:** `worlds/humanoid_world.world`

```xml
<?xml version="1.0"?>
<sdf version="1.6">
  <world name="humanoid_world">
    
    <!-- Physics settings -->
    <physics name="default_physics" default="true" type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
    </physics>
    
    <!-- Lighting -->
    <light name="sun" type="directional">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <attenuation>
        <range>1000</range>
        <constant>0.9</constant>
        <linear>0.01</linear>
        <quadratic>0.001</quadratic>
      </attenuation>
      <direction>-0.5 0.1 -0.9</direction>
    </light>
    
    <!-- Ground plane -->
    <include>
      <uri>model://ground_plane</uri>
    </include>
    
    <!-- Obstacles -->
    <model name="box_obstacle">
      <pose>2 0 0.5 0 0 0</pose>
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>1 1 1</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>1 1 1</size>
            </box>
          </geometry>
          <material>
            <ambient>0.8 0.2 0.2 1</ambient>
            <diffuse>0.8 0.2 0.2 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
    
    <!-- Stairs for testing bipedal locomotion -->
    <include>
      <uri>model://stairs</uri>
      <pose>-3 0 0 0 0 1.57</pose>
    </include>
    
  </world>
</sdf>
```

## Launching Gazebo

**File:** `launch/gazebo.launch.py`

```python
import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_gazebo_ros = FindPackageShare('gazebo_ros')
    world_file = os.path.join(
        get_package_share_directory('humanoid_simulation'),
        'worlds',
        'humanoid_world.world'
    )
    
    # Gazebo server
    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            pkg_gazebo_ros, '/launch/gzserver.launch.py'
        ]),
        launch_arguments={'world': world_file}.items()
    )
    
    # Gazebo client (GUI)
    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            pkg_gazebo_ros, '/launch/gzclient.launch.py'
        ])
    )
    
    return LaunchDescription([gzserver, gzclient])
```

## Spawning Robots

```python
from launch_ros.actions import Node

spawn_entity = Node(
    package='gazebo_ros',
    executable='spawn_entity.py',
    arguments=[
        '-entity', 'humanoid',
        '-file', urdf_path,
        '-x', '0', '-y', '0', '-z', '1.0',
        '-R', '0', '-P', '0', '-Y', '0'
    ],
    output='screen'
)
```

## Gazebo Plugins

### IMU Sensor Plugin

```xml
<gazebo reference="torso">
  <sensor name="imu_sensor" type="imu">
    <always_on>true</always_on>
    <update_rate>100</update_rate>
    <plugin name="imu_plugin" filename="libgazebo_ros_imu_sensor.so">
      <ros>
        <namespace>/imu</namespace>
        <remapping>~/out:=data</remapping>
      </ros>
      <initial_orientation_as_reference>false</initial_orientation_as_reference>
    </plugin>
  </sensor>
</gazebo>
```

### Camera Plugin

```xml
<gazebo reference="head">
  <sensor name="camera" type="camera">
    <camera>
      <horizontal_fov>1.047</horizontal_fov>
      <image>
        <width>640</width>
        <height>480</height>
        <format>R8G8B8</format>
      </image>
      <clip>
        <near>0.1</near>
        <far>100</far>
      </clip>
    </camera>
    <always_on>true</always_on>
    <update_rate>30</update_rate>
    <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
      <ros>
        <namespace>/camera</namespace>
        <remapping>image_raw:=rgb/image_raw</remapping>
        <remapping>camera_info:=rgb/camera_info</remapping>
      </ros>
      <camera_name>front_camera</camera_name>
      <frame_name>camera_link</frame_name>
    </plugin>
  </sensor>
</gazebo>
```

### Joint Control Plugin

```xml
<gazebo>
  <plugin name="gazebo_ros2_control" filename="libgazebo_ros2_control.so">
    <parameters>$(find humanoid_control)/config/controllers.yaml</parameters>
  </plugin>
</gazebo>
```

##Summary

✅ Gazebo setup and architecture  
✅ World creation  
✅ Robot spawning  
✅ Sensor plugins (IMU, camera)  

## Next Chapter

In **Chapter 7**, we'll dive deeper into:
- SDF (Simulation Description Format)
- Advanced model creation
- Plugin development

👉 [Continue to Chapter 7 →](./chapter-7.md)
