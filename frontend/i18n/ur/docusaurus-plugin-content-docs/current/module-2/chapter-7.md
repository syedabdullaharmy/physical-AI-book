---
sidebar_position: 7
---

# Chapter 7: URDF and SDF Formats

## Introduction

While URDF is standard in ROS, **SDF (Simulation Description Format)** is Gazebo's native format with more advanced features. Understanding both and knowing when to use each is essential for robot simulation.

:::tip Learning Objectives
- Understand differences between URDF and SDF
- Convert between formats
- Use SDF's advanced features
- Create complex simulation models
- Optimize for performance
:::

## URDF vs SDF: Key Differences

| Feature | URDF | SDF |
|---------|------|-----|
| **Scope** | Single robot | Entire worlds + multiple robots |
| **Plugins** | Limited | Extensive ecosystem |
| **Physics** | Basic properties | Advanced physics engines |
| **Sensors** | Via Gazebo plugins | Native support |
| **Nested Models** | No | Yes |
| **Closed Loops** | No | Yes |
| **Coordinate Frames** | Parent-child only | Arbitrary |

### When to Use URDF

✅ ROS-centric applications  
✅ Robot description for RViz  
✅ MoveIt motion planning  
✅ Simple robot models  
✅ Maximum ROS compatibility  

### When to Use SDF

✅ Complex simulation environments  
✅ Multiple interacting robots  
✅ Advanced sensor simulation  
✅ Physics plugin development  
✅ Gazebo-specific features  

## SDF Structure

### Basic SDF Model

**File:** `models/simple_robot/model.sdf`

```xml
<?xml version="1.0"?>
<sdf version="1.7">
  <model name="simple_robot">
    
    <!-- Model pose in world -->
    <pose>0 0 0.5 0 0 0</pose>
    
    <!-- Links -->
    <link name="base_link">
      <inertial>
        <mass>10.0</mass>
        <inertia>
          <ixx>0.83</ixx>
          <iyy>0.83</iyy>
          <izz>1.25</izz>
          <ixy>0</ixy>
          <ixz>0</ixz>
          <iyz>0</iyz>
        </inertia>
      </inertial>
      
      <collision name="collision">
        <geometry>
          <box>
            <size>0.5 0.5 0.25</size>
          </box>
        </geometry>
        <surface>
          <friction>
            <ode>
              <mu>0.8</mu>
              <mu2>0.8</mu2>
            </ode>
          </friction>
        </surface>
      </collision>
      
      <visual name="visual">
        <geometry>
          <box>
            <size>0.5 0.5 0.25</size>
          </box>
        </geometry>
        <material>
          <ambient>0.2 0.4 0.8 1</ambient>
          <diffuse>0.2 0.4 0.8 1</diffuse>
          <specular>0.5 0.5 0.5 1</specular>
        </material>
      </visual>
      
      <!-- Sensor (native in SDF) -->
      <sensor name="imu_sensor" type="imu">
        <always_on>true</always_on>
        <update_rate>100</update_rate>
        <imu>
          <angular_velocity>
            <x>
              <noise type="gaussian">
                <mean>0</mean>
                <stddev>0.01</stddev>
              </noise>
            </x>
            <y>
              <noise type="gaussian">
                <mean>0</mean>
                <stddev>0.01</stddev>
              </noise>
            </y>
            <z>
              <noise type="gaussian">
                <mean>0</mean>
                <stddev>0.01</stddev>
              </noise>
            </z>
          </angular_velocity>
          <linear_acceleration>
            <x>
              <noise type="gaussian">
                <mean>0</mean>
                <stddev>0.1</stddev>
              </noise>
            </x>
            <y>
              <noise type="gaussian">
                <mean>0</mean>
                <stddev>0.1</stddev>
              </noise>
            </y>
            <z>
              <noise type="gaussian">
                <mean>0</mean>
                <stddev>0.1</stddev>
              </noise>
            </z>
          </linear_acceleration>
        </imu>
      </sensor>
    </link>
    
    <!-- Joints -->
    <joint name="wheel_joint" type="revolute">
      <parent>base_link</parent>
      <child>wheel_link</child>
      <axis>
        <xyz>0 1 0</xyz>
        <limit>
          <lower>-1e16</lower>
          <upper>1e16</upper>
        </limit>
        <dynamics>
          <damping>0.1</damping>
          <friction>0.05</friction>
        </dynamics>
      </axis>
    </joint>
    
    <!-- Plugins -->
    <plugin name="diff_drive" filename="libgazebo_ros_diff_drive.so">
      <ros>
        <namespace>/robot</namespace>
      </ros>
      <left_joint>left_wheel_joint</left_joint>
      <right_joint>right_wheel_joint</right_joint>
      <wheel_separation>0.4</wheel_separation>
      <wheel_diameter>0.2</wheel_diameter>
      <max_wheel_torque>20</max_wheel_torque>
      <max_wheel_acceleration>1.0</max_wheel_acceleration>
      <publish_odom>true</publish_odom>
      <publish_odom_tf>true</publish_odom_tf>
      <publish_wheel_tf>true</publish_wheel_tf>
      <odometry_frame>odom</odometry_frame>
      <robot_base_frame>base_link</robot_base_frame>
    </plugin>
    
  </model>
</sdf>
```

## Converting Between URDF and SDF

### URDF to SDF Conversion

Gazebo automatically converts URDF to SDF internally:

```bash
# Method 1: Automatic (Gazebo does this)
ros2 launch gazebo_ros spawn_entity.py -file robot.urdf

# Method 2: Manual conversion
gz sdf -p robot.urdf > robot.sdf
```

### Conversion Script (Python)

```python
#!/usr/bin/env python3
import xml.etree.ElementTree as ET

def urdf_to_sdf(urdf_file, sdf_file):
    """Convert URDF to SDF format"""
    tree = ET.parse(urdf_file)
    root = tree.getroot()
    
    # Create SDF structure
    sdf = ET.Element('sdf', version='1.7')
    model = ET.SubElement(sdf, 'model', name=root.get('name'))
    
    # Convert links
    for link in root.findall('link'):
        sdf_link = ET.SubElement(model, 'link', name=link.get('name'))
        
        # Copy inertial
        inertial = link.find('inertial')
        if inertial is not None:
            sdf_link.append(inertial)
        
        # Copy visual
        visual = link.find('visual')
        if visual is not None:
            sdf_link.append(visual)
        
        # Copy collision
        collision = link.find('collision')
        if collision is not None:
            sdf_link.append(collision)
    
    # Convert joints
    for joint in root.findall('joint'):
        sdf_joint = ET.SubElement(model, 'joint',
                                  name=joint.get('name'),
                                  type=joint.get('type'))
        
        # Parent and child
        parent = joint.find('parent')
        child = joint.find('child')
        
        ET.SubElement(sdf_joint, 'parent').text = parent.get('link')
        ET.SubElement(sdf_joint, 'child').text = child.get('link')
        
        # Axis
        axis_elem = joint.find('axis')
        if axis_elem is not None:
            sdf_axis = ET.SubElement(sdf_joint, 'axis')
            xyz = axis_elem.find('xyz')
            ET.SubElement(sdf_axis, 'xyz').text = xyz.text
            
            # Limits
            limit = axis_elem.find('limit')
            if limit is not None:
                sdf_limit = ET.SubElement(sdf_axis, 'limit')
                for child_elem in limit:
                    sdf_limit.append(child_elem)
    
    # Write SDF file
    tree = ET.ElementTree(sdf)
    tree.write(sdf_file, encoding='utf-8', xml_declaration=True)
    print(f"Converted {urdf_file} to {sdf_file}")

# Usage
urdf_to_sdf('robot.urdf', 'robot.sdf')
```

## Advanced SDF Features

### 1. Nested Models

SDF supports model composition:

```xml
<model name="warehouse">
  <pose>0 0 0 0 0 0</pose>
  
  <!-- Include another model -->
  <include>
    <uri>model://table</uri>
    <pose>1 0 0 0 0 0</pose>
  </include>
  
  <include>
    <uri>model://robot_arm</uri>
    <pose>1 0 1 0 0 0</pose>
  </include>
  
  <!-- Nested model -->
  <model name="conveyor_belt">
    <pose>5 0 0.5 0 0 0</pose>
    <link name="belt">
      <!-- Belt definition -->
    </link>
  </model>
</model>
```

### 2. Advanced Physics

```xml
<link name="body">
  <collision name="collision">
    <geometry>
      <box><size>1 1 1</size></box>
    </geometry>
    
    <!-- Advanced surface properties -->
    <surface>
      <friction>
        <ode>
          <mu>0.8</mu>
          <mu2>0.6</mu2>
          <slip1>0.01</slip1>
          <slip2>0.01</slip2>
          <fdir1>1 0 0</fdir1>
        </ode>
        <bullet>
          <friction>0.8</friction>
          <friction2>0.6</friction2>
          <rolling_friction>0.01</rolling_friction>
        </bullet>
      </friction>
      
      <contact>
        <ode>
          <kp>1000000</kp>  <!-- Stiffness -->
          <kd>100</kd>      <!-- Damping -->
          <max_vel>0.01</max_vel>
          <min_depth>0.001</min_depth>
        </ode>
      </contact>
      
      <bounce>
        <restitution_coefficient>0.5</restitution_coefficient>
        <threshold>0.1</threshold>
      </bounce>
    </surface>
  </collision>
</link>
```

### 3. Sensor Arrays

```xml
<link name="sensor_head">
  <!-- LiDAR -->
  <sensor name="lidar" type="ray">
    <pose>0 0 0.1 0 0 0</pose>
    <ray>
      <scan>
        <horizontal>
          <samples>640</samples>
          <resolution>1</resolution>
          <min_angle>-1.57</min_angle>
          <max_angle>1.57</max_angle>
        </horizontal>
        <vertical>
          <samples>16</samples>
          <resolution>1</resolution>
          <min_angle>-0.26</min_angle>
          <max_angle>0.26</max_angle>
        </vertical>
      </scan>
      <range>
        <min>0.1</min>
        <max>30.0</max>
        <resolution>0.01</resolution>
      </range>
    </ray>
    <always_on>true</always_on>
    <update_rate>10</update_rate>
  </sensor>
  
  <!-- Depth Camera -->
  <sensor name="depth_camera" type="depth">
    <camera>
      <horizontal_fov>1.047</horizontal_fov>
      <image>
        <width>640</width>
        <height>480</height>
      </image>
      <clip>
        <near>0.1</near>
        <far>10</far>
      </clip>
    </camera>
    <always_on>true</always_on>
    <update_rate>30</update_rate>
  </sensor>
</link>
```

## Complete Humanoid in SDF

**File:** `models/humanoid_sdf/model.sdf`

```xml
<?xml version="1.0"?>
<sdf version="1.7">
  <model name="humanoid_sdf">
    <pose>0 0 1.0 0 0 0</pose>
    <self_collide>true</self_collide>
    
    <!-- Base link -->
    <link name="base_link">
      <inertial>
        <mass>5.0</mass>
        <inertia>
          <ixx>0.05</ixx>
          <iyy>0.08</iyy>
          <izz>0.08</izz>
        </inertia>
      </inertial>
      <collision name="collision">
        <geometry>
          <box><size>0.3 0.2 0.1</size></box>
        </geometry>
      </collision>
      <visual name="visual">
        <geometry>
          <box><size>0.3 0.2 0.1</size></box>
        </geometry>
        <material>
          <ambient>0.5 0.5 0.5 1</ambient>
        </material>
      </visual>
      
      <!-- IMU Sensor -->
      <sensor name="imu" type="imu">
        <always_on>true</always_on>
        <update_rate>100</update_rate>
      </sensor>
    </link>
    
    <!-- Contact sensors on feet -->
    <link name="left_foot">
      <sensor name="left_contact" type="contact">
        <contact>
          <collision>left_foot_collision</collision>
        </contact>
        <always_on>true</always_on>
        <update_rate>100</update_rate>
      </sensor>
    </link>
    
    <!-- Gazebo ROS2 Control Plugin -->
    <plugin name="gazebo_ros2_control" filename="libgaz ebo_ros2_control.so">
      <parameters>config/controllers.yaml</parameters>
    </plugin>
    
  </model>
</sdf>
```

## Model Organization

### Directory Structure

```
models/
├── humanoid/
│   ├── model.config
│   ├── model.sdf
│   ├── meshes/
│   │   ├── head.dae
│   │   ├── torso.dae
│   │   └── limbs.dae
│   ├── materials/
│   │   ├── scripts/
│   │   └── textures/
│   └── thumbnails/
│       └── thumbnail.png
```

### model.config

```xml
<?xml version="1.0"?>
<model>
  <name>Humanoid Robot</name>
  <version>1.0</version>
  <sdf version="1.7">model.sdf</sdf>
  
  <author>
    <name>Your Name</name>
    <email>you@example.com</email>
  </author>
  
  <description>
    17-DOF humanoid robot for research and education
  </description>
</model>
```

## Performance Optimization

### 1. Simplified Collision Meshes

```xml
<collision name="collision">
  <geometry>
    <!-- Use simple shapes for collision -->
    <box><size>0.3 0.2 0.6</size></box>
  </geometry>
</collision>

<visual name="visual">
  <geometry>
    <!-- Use detailed mesh for visualization -->
    <mesh><uri>model://humanoid/meshes/torso.dae</uri></mesh>
  </geometry>
</visual>
```

### 2. Level of Detail (LOD)

```xml
<visual name="visual_lod">
  <geometry>
    <mesh>
      <uri>model://humanoid/meshes/torso_high.dae</uri>
    </mesh>
  </geometry>
  <plugin name="lod" filename="libLODPlugin.so">
    <distances>10 50 100</distances>
    <meshes>
      <mesh>torso_high.dae</mesh>
      <mesh>torso_medium.dae</mesh>
      <mesh>torso_low.dae</mesh>
    </meshes>
  </plugin>
</visual>
```

## Summary

✅ URDF vs SDF differences and use cases  
✅ Format conversion techniques  
✅ Advanced SDF features (nested models, sensors)  
✅ Physics optimization  
✅ Model organization  

## Practice Exercises

1. Convert your humanoid URDF to SDF
2. Add contact sensors to robot feet
3. Create nested warehouse model
4. Optimize collision meshes

## Next Chapter

In **Chapter 8**, we'll explore:
- Physics engines in depth
- Sensor simulation techniques
- Performance tuning

👉 [Continue to Chapter 8 →](./chapter-8.md)
