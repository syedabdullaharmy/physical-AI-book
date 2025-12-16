---
sidebar_position: 5
---

import ChapterToolbar from '@site/src/components/ChapterToolbar';

<ChapterToolbar 
    chapterId="module-1/chapter-5" 
    chapterTitle="URDF for Humanoid Robots" 
/>

# Chapter 5: URDF for Humanoid Robots

## Introduction

The **Unified Robot Description Format (URDF)** is the standard way to describe robot geometry, kinematics, and dynamics in ROS. In this chapter, you'll learn to model a complete 17-DOF humanoid robot.

:::tip Learning Objectives
- Understand URDF structure (links, joints, transforms)
- Create visual and collision geometries
- Define joint limits and dynamics
- Use Xacro for modular robot descriptions
- Visualize robots in RViz
:::

## URDF Basics

### Core Components

URDF files describe robots using two main elements:

1. **Links** - Rigid bodies (bones, chassis, sensors)
2. **Joints** - Connections between links (revolute, prismatic, fixed)

```xml
<robot name="simple_arm">
  <link name="base_link"/>
  
  <link name="upper_arm"/>
  <joint name="shoulder" type="revolute">
    <parent link="base_link"/>
    <child link="upper_arm"/>
    <origin xyz="0 0 0.1" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
  </joint>
</robot>
```

### Link Properties

```xml
<link name="upper_arm">
  <!-- Visual (what you see) -->
  <visual>
    <geometry>
      <cylinder radius="0.05" length="0.4"/>
    </geometry>
    <material name="blue">
      <color rgba="0 0 0.8 1"/>
    </material>
  </visual>
  
  <!-- Collision (for physics) -->
  <collision>
    <geometry>
      <cylinder radius="0.05" length="0.4"/>
    </geometry>
  </collision>
  
  <!-- Inertial (mass and inertia) -->
  <inertial>
    <mass value="1.5"/>
    <inertia ixx="0.0201" ixy="0" ixz="0"
             iyy="0.0201" iyz="0" izz="0.00188"/>
  </inertial>
</link>
```

### Joint Types

| Type | Description | DOF |
|------|-------------|-----|
| **revolute** | Rotating joint with limits | 1 |
| **continuous** | Rotating joint without limits | 1 |
| **prismatic** | Sliding joint | 1 |
| **fixed** | No motion | 0 |
| **floating** | 6-DOF (position + orientation) | 6 |
| **planar** | Motion in a plane | 2 |

## Building a Humanoid Robot

### Humanoid Structure (17 DOF)

```
         HEAD (2 DOF: pan, tilt)
            |
         TORSO
        /   |   \
      L-ARM  |  R-ARM (3 DOF each)
             |
          WAIST (2 DOF: yaw, pitch)
            |
        /       \
    L-LEG      R-LEG (4 DOF each)
```

### Complete Humanoid URDF

**File:** `urdf/humanoid.urdf`

```xml
<?xml version="1.0"?>
<robot name="humanoid_alpha">
  
  <!-- BASE LINK (root of kinematic  tree) -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.3 0.2 0.1"/>
      </geometry>
      <material name="grey">
        <color rgba="0.5 0.5 0.5 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.3 0.2 0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="5.0"/>
      <inertia ixx="0.05" ixy="0" ixz="0"
               iyy="0.08" iyz="0" izz="0.08"/>
    </inertial>
  </link>
  
  <!-- TORSO -->
  <link name="torso">
    <visual>
      <geometry>
        <box size="0.3 0.2 0.6"/>
      </geometry>
      <material name="white">
        <color rgba="0.9 0.9 0.9 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.3 0.2 0.6"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="10.0"/>
      <inertia ixx="0.35" ixy="0" ixz="0"
               iyy="0.37" iyz="0" izz="0.08"/>
    </inertial>
  </link>
  
  <joint name="base_to_torso" type="fixed">
    <parent link="base_link"/>
    <child link="torso"/>
    <origin xyz="0 0 0.35" rpy="0 0 0"/>
  </joint>
  
  <!-- HEAD -->
  <link name="head_base">
    <visual>
      <geometry>
        <sphere radius="0.12"/>
      </geometry>
      <material name="skin">
        <color rgba="0.95 0.8 0.7 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <sphere radius="0.12"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="2.0"/>
      <inertia ixx="0.0038" ixy="0" ixz="0"
               iyy="0.0038" iyz="0" izz="0.0038"/>
    </inertial>
  </link>
  
  <joint name="head_pan" type="revolute">
    <parent link="torso"/>
    <child link="head_base"/>
    <origin xyz="0 0 0.35" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.57" upper="1.57" effort="50" velocity="2.0"/>
    <dynamics damping="0.7"/>
  </joint>
  
  <link name="head">
    <visual>
      <geometry>
        <box size="0.15 0.12 0.18"/>
      </geometry>
      <material name="skin"/>
    </visual>
    <inertial>
      <mass value="1.5"/>
      <inertia ixx="0.0027" ixy="0" ixz="0"
               iyy="0.0032" iyz="0" izz="0.0023"/>
    </inertial>
  </link>
  
  <joint name="head_tilt" type="revolute">
    <parent link="head_base"/>
    <child link="head"/>
    <origin xyz="0 0 0.08" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-0.79" upper="0.79" effort="30" velocity="2.0"/>
  </joint>
  
  <!-- LEFT ARM -->
  <link name="left_shoulder">
    <visual>
      <geometry>
        <cylinder radius="0.05" length="0.1"/>
      </geometry>
      <material name="blue">
        <color rgba="0.2 0.4 0.8 1"/>
      </material>
    </visual>
    <inertial>
      <mass value="0.5"/>
      <inertia ixx="0.0005" ixy="0" ixz="0"
               iyy="0.0005" iyz="0" izz="0.000125"/>
    </inertial>
  </link>
  
  <joint name="left_shoulder_pitch" type="revolute">
    <parent link="torso"/>
    <child link="left_shoulder"/>
    <origin xyz="0 0.15 0.25" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-2.09" upper="2.09" effort="100" velocity="2.0"/>
  </joint>
  
  <link name="left_upper_arm">
    <visual>
      <origin xyz="0 0 -0.15" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.04" length="0.3"/>
      </geometry>
      <material name="blue"/>
    </visual>
    <inertial>
      <mass value="1.2"/>
      <origin xyz="0 0 -0.15" rpy="0 0 0"/>
      <inertia ixx="0.0091" ixy="0" ixz="0"
               iyy="0.0091" iyz="0" izz="0.00096"/>
    </inertial>
  </link>
  
  <joint name="left_shoulder_roll" type="revolute">
    <parent link="left_shoulder"/>
    <child link="left_upper_arm"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <axis xyz="1 0 0"/>
    <limit lower="-1.57" upper="1.57" effort="80" velocity="2.0"/>
  </joint>
  
  <link name="left_forearm">
    <visual>
      <origin xyz="0 0 -0.125" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.03" length="0.25"/>
      </geometry>
      <material name="white"/>
    </visual>
    <inertial>
      <mass value="0.8"/>
      <origin xyz="0 0 -0.125" rpy="0 0 0"/>
      <inertia ixx="0.0042" ixy="0" ixz="0"
               iyy="0.0042" iyz="0" izz="0.00036"/>
    </inertial>
  </link>
  
  <joint name="left_elbow" type="revolute">
    <parent link="left_upper_arm"/>
    <child link="left_forearm"/>
    <origin xyz="0 0 -0.3" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="0" upper="2.62" effort="60" velocity="2.0"/>
  </joint>
  
  <!-- RIGHT ARM (mirror of left) -->
  <link name="right_shoulder">
    <visual>
      <geometry>
        <cylinder radius="0.05" length="0.1"/>
      </geometry>
      <material name="blue"/>
    </visual>
    <inertial>
      <mass value="0.5"/>
      <inertia ixx="0.0005" ixy="0" ixz="0"
               iyy="0.0005" iyz="0" izz="0.000125"/>
    </inertial>
  </link>
  
  <joint name="right_shoulder_pitch" type="revolute">
    <parent link="torso"/>
    <child link="right_shoulder"/>
    <origin xyz="0 -0.15 0.25" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-2.09" upper="2.09" effort="100" velocity="2.0"/>
  </joint>
  
  <link name="right_upper_arm">
    <visual>
      <origin xyz="0 0 -0.15" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.04" length="0.3"/>
      </geometry>
      <material name="blue"/>
    </visual>
    <inertial>
      <mass value="1.2"/>
      <origin xyz="0 0 -0.15" rpy="0 0 0"/>
      <inertia ixx="0.0091" ixy="0" ixz="0"
               iyy="0.0091" iyz="0" izz="0.00096"/>
    </inertial>
  </link>
  
  <joint name="right_shoulder_roll" type="revolute">
    <parent link="right_shoulder"/>
    <child link="right_upper_arm"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <axis xyz="1 0 0"/>
    <limit lower="-1.57" upper="1.57" effort="80" velocity="2.0"/>
  </joint>
  
  <link name="right_forearm">
    <visual>
      <origin xyz="0 0 -0.125" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.03" length="0.25"/>
      </geometry>
      <material name="white"/>
    </visual>
    <inertial>
      <mass value="0.8"/>
      <origin xyz="0 0 -0.125" rpy="0 0 0"/>
      <inertia ixx="0.0042" ixy="0" ixz="0"
               iyy="0.0042" iyz="0" izz="0.00036"/>
    </inertial>
  </link>
  
  <joint name="right_elbow" type="revolute">
    <parent link="right_upper_arm"/>
    <child link="right_forearm"/>
    <origin xyz="0 0 -0.3" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="0" upper="2.62" effort="60" velocity="2.0"/>
  </joint>
  
  <!-- WAIST -->
  <link name="waist_yaw">
    <visual>
      <geometry>
        <cylinder radius="0.08" length="0.1"/>
      </geometry>
      <material name="grey"/>
    </visual>
    <inertial>
      <mass value="2.0"/>
      <inertia ixx="0.00133" ixy="0" ixz="0"
               iyy="0.00133" iyz="0" izz="0.00128"/>
    </inertial>
  </link>
  
  <joint name="waist_yaw" type="revolute">
    <parent link="base_link"/>
    <child link="waist_yaw"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.57" upper="1.57" effort="150" velocity="1.0"/>
  </joint>
  
  <joint name="waist_pitch" type="revolute">
    <parent link="waist_yaw"/>
    <child link="torso"/>
    <origin xyz="0 0 0.05" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-0.79" upper="0.79" effort="150" velocity="1.0"/>
  </joint>
  
  <!-- LEFT LEG (4 DOF: hip pitch, hip roll, knee, ankle) -->
  <link name="left_hip">
    <visual>
      <geometry>
        <sphere radius="0.06"/>
      </geometry>
      <material name="grey"/>
    </visual>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.00144" ixy="0" ixz="0"
               iyy="0.00144" iyz="0" izz="0.00144"/>
    </inertial>
  </link>
  
  <joint name="left_hip_pitch" type="revolute">
    <parent link="base_link"/>
    <child link="left_hip"/>
    <origin xyz="0 0.1 -0.05" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="200" velocity="2.0"/>
  </joint>
  
  <link name="left_thigh">
    <visual>
      <origin xyz="0 0 -0.2" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.06" length="0.4"/>
      </geometry>
      <material name="blue"/>
    </visual>
    <inertial>
      <mass value="3.0"/>
      <origin xyz="0 0 -0.2" rpy="0 0 0"/>
      <inertia ixx="0.0405" ixy="0" ixz="0"
               iyy="0.0405" iyz="0" izz="0.0054"/>
    </inertial>
  </link>
  
  <joint name="left_hip_roll" type="revolute">
    <parent link="left_hip"/>
    <child link="left_thigh"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <axis xyz="1 0 0"/>
    <limit lower="-0.79" upper="0.79" effort="180" velocity="2.0"/>
  </joint>
  
  <link name="left_shin">
    <visual>
      <origin xyz="0 0 -0.175" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.04" length="0.35"/>
      </geometry>
      <material name="white"/>
    </visual>
    <inertial>
      <mass value="2.0"/>
      <origin xyz="0 0 -0.175" rpy="0 0 0"/>
      <inertia ixx="0.0205" ixy="0" ixz="0"
               iyy="0.0205" iyz="0" izz="0.0016"/>
    </inertial>
  </link>
  
  <joint name="left_knee" type="revolute">
    <parent link="left_thigh"/>
    <child link="left_shin"/>
    <origin xyz="0 0 -0.4" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="0" upper="2.62" effort="150" velocity="2.0"/>
  </joint>
  
  <link name="left_foot">
    <visual>
      <geometry>
        <box size="0.2 0.1 0.05"/>
      </geometry>
      <material name="grey"/>
    </visual>
    <collision>
      <geometry>
        <box size="0.2 0.1 0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.5"/>
      <inertia ixx="0.00042" ixy="0" ixz="0"
               iyy="0.00175" iyz="0" izz="0.00188"/>
    </inertial>
  </link>
  
  <joint name="left_ankle" type="revolute">
    <parent link="left_shin"/>
    <child link="left_foot"/>
    <origin xyz="0 0 -0.35" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-0.79" upper="0.79" effort="100" velocity="2.0"/>
  </joint>
  
  <!-- RIGHT LEG (mirror of left) -->
  <link name="right_hip">
    <visual>
      <geometry>
        <sphere radius="0.06"/>
      </geometry>
      <material name="grey"/>
    </visual>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.00144" ixy="0" ixz="0"
               iyy="0.00144" iyz="0" izz="0.00144"/>
    </inertial>
  </link>
  
  <joint name="right_hip_pitch" type="revolute">
    <parent link="base_link"/>
    <child link="right_hip"/>
    <origin xyz="0 -0.1 -0.05" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="200" velocity="2.0"/>
  </joint>
  
  <link name="right_thigh">
    <visual>
      <origin xyz="0 0 -0.2" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.06" length="0.4"/>
      </geometry>
      <material name="blue"/>
    </visual>
    <inertial>
      <mass value="3.0"/>
      <origin xyz="0 0 -0.2" rpy="0 0 0"/>
      <inertia ixx="0.0405" ixy="0" ixz="0"
               iyy="0.0405" iyz="0" izz="0.0054"/>
    </inertial>
  </link>
  
  <joint name="right_hip_roll" type="revolute">
    <parent link="right_hip"/>
    <child link="right_thigh"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <axis xyz="1 0 0"/>
    <limit lower="-0.79" upper="0.79" effort="180" velocity="2.0"/>
  </joint>
  
  <link name="right_shin">
    <visual>
      <origin xyz="0 0 -0.175" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.04" length="0.35"/>
      </geometry>
      <material name="white"/>
    </visual>
    <inertial>
      <mass value="2.0"/>
      <origin xyz="0 0 -0.175" rpy="0 0 0"/>
      <inertia ixx="0.0205" ixy="0" ixz="0"
               iyy="0.0205" iyz="0" izz="0.0016"/>
    </inertial>
  </link>
  
  <joint name="right_knee" type="revolute">
    <parent link="right_thigh"/>
    <child link="right_shin"/>
    <origin xyz="0 0 -0.4" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="0" upper="2.62" effort="150" velocity="2.0"/>
  </joint>
  
  <link name="right_foot">
    <visual>
      <geometry>
        <box size="0.2 0.1 0.05"/>
      </geometry>
      <material name="grey"/>
    </visual>
    <collision>
      <geometry>
        <box size="0.2 0.1 0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.5"/>
      <inertia ixx="0.00042" ixy="0" ixz="0"
               iyy="0.00175" iyz="0" izz="0.00188"/>
    </inertial>
  </link>
  
  <joint name="right_ankle" type="revolute">
    <parent link="right_shin"/>
    <child link="right_foot"/>
    <origin xyz="0 0 -0.35" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-0.79" upper="0.79" effort="100" velocity="2.0"/>
  </joint>
  
</robot>
```

## Xacro: Macros for URDF

**Xacro** (XML Macros) makes URDF more maintainable:

### Basic Xacro Example

**File:** `urdf/humanoid.urdf.xacro`

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="humanoid">
  
  <!-- Properties (constants) -->
  <xacro:property name="arm_length" value="0.3"/>
  <xacro:property name="leg_length" value="0.4"/>
  <xacro:property name="pi" value="3.14159265"/>
  
  <!-- Macro for creating a limb -->
  <xacro:macro name="limb" params="name length radius mass">
    <link name="${name}">
      <visual>
        <origin xyz="0 0 ${-length/2}" rpy="0 0 0"/>
        <geometry>
          <cylinder radius="${radius}" length="${length}"/>
        </geometry>
      </visual>
      <inertial>
        <mass value="${mass}"/>
        <inertia ixx="${mass*(3*radius*radius + length*length)/12}" 
                 iyy="${mass*(3*radius*radius + length*length)/12}"
                 izz="${mass*radius*radius/2}"
                 ixy="0" ixz="0" iyz="0"/>
      </inertial>
    </link>
  </xacro:macro>
  
  <!-- Use macro -->
  <xacro:limb name="left_upper_arm" length="${arm_length}" 
              radius="0.04" mass="1.2"/>
  <xacro:limb name="right_upper_arm" length="${arm_length}"
              radius="0.04" mass="1.2"/>
  
</robot>
```

**Convert Xacro to URDF:**
```bash
xacro humanoid.urdf.xacro > humanoid.urdf
```

## Visualization in RViz

### Display Robot

```bash
# Terminal 1: Publish robot state
ros2 run robot_state_publisher robot_state_publisher \
    --ros-args -p robot_description:="$(cat humanoid.urdf)"

# Terminal 2: Publish joint states
ros2 run joint_state_publisher_gui joint_state_publisher_gui

# Terminal 3: Launch RViz
rviz2
```

### RViz Configuration

1. Add "RobotModel" display
2. Set "Fixed Frame" to "base_link"
3. Adjust joint sliders to move robot

## Summary

✅ URDF structure (links, joints, inertia)  
✅ Complete 17-DOF humanoid model  
✅ Xacro for modular descriptions  
✅ RViz visualization  

## Next Chapter

Module 2 begins! In **Chapter 6**, we'll explore:
- Gazebo simulation environment
- Physics engines
- Sensor plugins
- Simulating our humanoid robot

👉 [Continue to Module 2: Chapter 6 →](../module-2/chapter-6.md)

---

:::tip Module 1 Complete!
You now understand ROS 2 fundamentals, package development, and robot modeling. Ready for simulation!
:::
