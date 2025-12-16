---
sidebar_position: 8
---

import ChapterToolbar from '@site/src/components/ChapterToolbar';

<ChapterToolbar 
    chapterId="module-2/chapter-8" 
    chapterTitle="Physics and Sensor Simulation" 
/>

# Chapter 8: Physics and Sensor Simulation

## Introduction

Realistic physics and sensor simulation are critical for developing robust robots. In this chapter, you'll learn to configure physics engines, simulate contact dynamics, and create realistic sensor models.

:::tip Learning Objectives
- Configure physics engines (ODE, Bullet, DART)
- Simulate contact and friction
- Model realistic sensors with noise
- Optimize simulation performance
- Validate sim-to-real transfer
:::

## Physics Engines in Gazebo

Gazebo supports multiple physics engines:

| Engine | Strengths | Use Cases |
|--------|-----------|-----------|
| **ODE** | Default, fast, stable | General robotics |
| **Bullet** | Accurate contacts | Manipulation |
| **DART** | Precise dynamics | Research |
| **Simbody** | Biomechanics | Humanoids |

### Selecting a Physics Engine

```xml
<world name="default">
  <physics name="default_physics" default="true" type="ode">
    <max_step_size>0.001</max_step_size>
    <real_time_factor>1.0</real_time_factor>
    <real_time_update_rate>1000</real_time_update_rate>
  </physics>
</world>
```

**Change to Bullet:**
```xml
<physics name="bullet_physics" type="bullet">
  <max_step_size>0.001</max_step_size>
  <bullet>
    <solver>
      <type>sequential_impulse</type>
      <iters>50</iters>
      <sor>1.3</sor>
    </solver>
  </bullet>
</physics>
```

## Contact Dynamics

### Surface Properties

```xml
<collision name="foot_collision">
  <geometry>
    <box><size>0.2 0.1 0.05</size></box>
  </geometry>
  <surface>
    <!-- Friction coefficients -->
    <friction>
      <ode>
        <mu>1.0</mu>      <!-- Primary friction -->
        <mu2>0.8</mu2>    <!-- Secondary friction -->
        <fdir1>1 0 0</fdir1> <!-- Friction direction -->
        <slip1>0.0</slip1>
        <slip2>0.0</slip2>
      </ode>
    </friction>
    
    <!-- Contact stiffness and damping -->
    <contact>
      <ode>
        <kp>1000000.0</kp>  <!-- Stiffness -->
        <kd>100.0</kd>       <!-- Damping -->
        <max_vel>0.01</max_vel>
        <min_depth>0.001</min_depth>
      </ode>
    </contact>
    
    <!-- Bounce properties -->
    <bounce>
      <restitution_coefficient>0.0</restitution_coefficient>
      <threshold>0.01</threshold>
    </bounce>
  </surface>
</collision>
```

### Contact Sensor

```xml
<sensor name="foot_contact" type="contact">
  <contact>
    <collision>left_foot_collision</collision>
  </contact>
  <always_on>true</always_on>
  <update_rate>100</update_rate>
  <plugin name="contact_plugin" filename="libgazebo_ros_bumper.so">
    <ros>
      <namespace>/humanoid</namespace>
      <remapping>bumper_states:=left_foot/contact</remapping>
    </ros>
    <frame_name>left_foot</frame_name>
  </plugin>
</sensor>
```

## IMU Simulation

### Realistic IMU with Noise

```xml
<sensor name="imu_sensor" type="imu">
  <always_on>true</always_on>
  <update_rate>100</update_rate>
  <imu>
    <!-- Angular velocity (gyroscope) -->
    <angular_velocity>
      <x>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>0.009</stddev>  <!-- 0.5 deg/s -->
          <bias_mean>0.00075</bias_mean>
          <bias_stddev>0.0000008</bias_stddev>
        </noise>
      </x>
      <y>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>0.009</stddev>
          <bias_mean>0.00075</bias_mean>
          <bias_stddev>0.0000008</bias_stddev>
        </noise>
      </y>
      <z>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>0.009</stddev>
          <bias_mean>0.00075</bias_mean>
          <bias_stddev>0.0000008</bias_stddev>
        </noise>
      </z>
    </angular_velocity>
    
    <!-- Linear acceleration (accelerometer) -->
    <linear_acceleration>
      <x>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>0.017</stddev>  <!-- 17 mg -->
          <bias_mean>0.1</bias_mean>
          <bias_stddev>0.001</bias_stddev>
        </noise>
      </x>
      <y>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>0.017</stddev>
          <bias_mean>0.1</bias_mean>
          <bias_stddev>0.001</bias_stddev>
        </noise>
      </y>
      <z>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>0.017</stddev>
          <bias_mean>0.1</bias_mean>
          <bias_stddev>0.001</bias_stddev>
        </noise>
      </z>
    </linear_acceleration>
  </imu>
  
  <plugin name="imu_plugin" filename="libgazebo_ros_imu_sensor.so">
    <ros>
      <namespace>/imu</namespace>
      <remapping>~/out:=data</remapping>
    </ros>
    <frame_name>imu_link</frame_name>
    <initial_orientation_as_reference>false</initial_orientation_as_reference>
  </plugin>
</sensor>
```

### Processing IMU Data

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import numpy as np

class ImuFilter(Node):
    def __init__(self):
        super().__init__('imu_filter')
        
        self.subscription = self.create_subscription(
            Imu, '/imu/data', self.imu_callback, 10)
        
        # Complementary filter state
        self.roll = 0.0
        self.pitch = 0.0
        self.alpha = 0.98  # Filter coefficient
        
    def imu_callback(self, msg):
        # Extract accelerometer data
        ax = msg.linear_acceleration.x
        ay = msg.linear_acceleration.y
        az = msg.linear_acceleration.z
        
        # Calculate roll and pitch from accelerometer
        accel_roll = np.arctan2(ay, az)
        accel_pitch = np.arctan2(-ax, np.sqrt(ay**2 + az**2))
        
        # Extract gyroscope data
        gx = msg.angular_velocity.x
        gy = msg.angular_velocity.y
        
        dt = 0.01  # 100 Hz
        
        # Complementary filter
        self.roll = self.alpha * (self.roll + gx * dt) + \
                    (1 - self.alpha) * accel_roll
        self.pitch = self.alpha * (self.pitch + gy * dt) + \
                     (1 - self.alpha) * accel_pitch
        
        self.get_logger().info(
            f'Roll: {np.degrees(self.roll):.2f}°, '
            f'Pitch: {np.degrees(self.pitch):.2f}°'
        )
```

## Camera Simulation

### RGB Camera

```xml
<sensor name="camera" type="camera">
  <pose>0 0 0.1 0 0 0</pose>
  <camera>
    <horizontal_fov>1.047</horizontal_fov>  <!-- 60 degrees -->
    <image>
      <width>640</width>
      <height>480</height>
      <format>R8G8B8</format>
    </image>
    <clip>
      <near>0.1</near>
      <far>100</far>
    </clip>
    
    <!-- Image noise -->
    <noise>
      <type>gaussian</type>
      <mean>0.0</mean>
      <stddev>0.007</stddev>
    </noise>
    
    <!-- Lens distortion -->
    <distortion>
      <k1>-0.1</k1>
      <k2>0.05</k2>
      <k3>-0.01</k3>
      <p1>0.001</p1>
      <p2>-0.001</p2>
      <center>0.5 0.5</center>
    </distortion>
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
```

### Depth Camera

```xml
<sensor name="depth_camera" type="depth">
  <camera>
    <horizontal_fov>1.047</horizontal_fov>
    <image>
      <width>640</width>
      <height>480</height>
    </image>
    <clip>
      <near>0.1</near>
      <far>10.0</far>
    </clip>
  </camera>
  
  <plugin name="depth_camera_controller" filename="libgazebo_ros_camera.so">
    <ros>
      <namespace>/depth_camera</namespace>
      <remapping>image_raw:=depth/image_raw</remapping>
      <remapping>camera_info:=depth/camera_info</remapping>
      <remapping>points:=depth/points</remapping>
    </ros>
    <frame_name>depth_camera_link</frame_name>
    <min_depth>0.1</min_depth>
    <max_depth>10.0</max_depth>
  </plugin>
</sensor>
```

## LiDAR Simulation

### 2D LiDAR

```xml
<sensor name="lidar_2d" type="ray">
  <pose>0 0 0.1 0 0 0</pose>
  <ray>
    <scan>
      <horizontal>
        <samples>720</samples>
        <resolution>1</resolution>
        <min_angle>-3.14159</min_angle>
        <max_angle>3.14159</max_angle>
      </horizontal>
    </scan>
    <range>
      <min>0.12</min>
      <max>30.0</max>
      <resolution>0.01</resolution>
    </range>
    <noise>
      <type>gaussian</type>
      <mean>0.0</mean>
      <stddev>0.01</stddev>
    </noise>
  </ray>
  
  <always_on>true</always_on>
  <update_rate>10</update_rate>
  
  <plugin name="lidar_controller" filename="libgazebo_ros_ray_sensor.so">
    <ros>
      <namespace>/lidar</namespace>
      <remapping>~/out:=scan</remapping>
    </ros>
    <frame_name>lidar_link</frame_name>
    <output_type>sensor_msgs/LaserScan</output_type>
  </plugin>
</sensor>
```

### 3D LiDAR (Velodyne-style)

```xml
<sensor name="lidar_3d" type="ray">
  <ray>
    <scan>
      <horizontal>
        <samples>1800</samples>
        <resolution>1</resolution>
        <min_angle>-3.14159</min_angle>
        <max_angle>3.14159</max_angle>
      </horizontal>
      <vertical>
        <samples>16</samples>
        <resolution>1</resolution>
        <min_angle>-0.2618</min_angle>
        <max_angle>0.2618</max_angle>
      </vertical>
    </scan>
    <range>
      <min>0.1</min>
      <max>100.0</max>
      <resolution>0.01</resolution>
    </range>
  </ray>
  
  <plugin name="lidar_3d_controller" filename="libgazebo_ros_ray_sensor.so">
    <output_type>sensor_msgs/PointCloud2</output_type>
    <frame_name>lidar_3d_link</frame_name>
  </plugin>
</sensor>
```

## GPS Simulation

```xml
<sensor name="gps" type="gps">
  <always_on>true</always_on>
  <update_rate>10</update_rate>
  <gps>
    <position_sensing>
      <horizontal>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>2.0</stddev>  <!-- 2m accuracy -->
        </noise>
      </horizontal>
      <vertical>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>4.0</stddev>  <!-- 4m accuracy -->
        </noise>
      </vertical>
    </position_sensing>
  </gps>
  
  <plugin name="gps_plugin" filename="libgazebo_ros_gps_sensor.so">
    <ros>
      <namespace>/gps</namespace>
      <remapping>~/out:=fix</remapping>
    </ros>
    <frame_name>gps_link</frame_name>
  </plugin>
</sensor>
```

## Force/Torque Sensors

```xml
<sensor name="wrist_force_torque" type="force_torque">
  <always_on>true</always_on>
  <update_rate>100</update_rate>
  <force_torque>
    <frame>child</frame>  <!-- Measure forces on child link -->
    <measure_direction>child_to_parent</measure_direction>
  </force_torque>
  
  <plugin name="ft_plugin" filename="libgazebo_ros_ft_sensor.so">
    <ros>
      <namespace>/wrist</namespace>
      <remapping>~/out:=force_torque</remapping>
    </ros>
    <frame_name>wrist_link</frame_name>
  </plugin>
</sensor>
```

## Performance Optimization

### 1. Physics Step Size

```xml
<physics type="ode">
  <!-- Smaller = more accurate but slower -->
  <max_step_size>0.001</max_step_size>
  
  <!-- Real-time factor -->
  <real_time_factor>1.0</real_time_factor>
  <real_time_update_rate>1000</real_time_update_rate>
  
  <!-- Solver settings -->
  <ode>
    <solver>
      <type>quick</type>  <!-- or 'world' for accuracy -->
      <iters>50</iters>
      <sor>1.3</sor>
    </solver>
  </ode>
</physics>
```

### 2. Collision Optimization

```python
# Use simple collision shapes
<collision name="simplified">
  <geometry>
    <!-- Use boxes/cylinders instead of meshes -->
    <box><size>0.5 0.3 0.2</size></box>
  </geometry>
</collision>

<visual name="detailed">
  <geometry>
    <!-- Detailed mesh for visualization -->
    <mesh><uri>model://robot/meshes/body.dae</uri></mesh>
  </geometry>
</visual>
```

### 3. Sensor Update Rates

Balance accuracy vs performance:

```
Camera: 30 Hz (adequate for vision)
IMU: 100-200 Hz (for control)
LiDAR: 10-20 Hz (environment mapping)
Contact: 100-500 Hz (for precise control)
```

## Validation Techniques

### 1. Compare with Analytical Solutions

```python
def validate_free_fall():
    """Validate gravity simulation"""
    # Drop object from height h
    h = 10.0  # meters
    g = 9.81  # m/s^2
    
    # Analytical solution
    t_analytical = np.sqrt(2 * h / g)
    
    # Measure in simulation
    t_simulated = measure_fall_time()
    
    error = abs(t_analytical - t_simulated) / t_analytical
    assert error < 0.01, f"Gravity error: {error*100:.2f}%"
```

### 2. Energy Conservation

```python
def check_energy_conservation():
    """Verify energy is conserved in frictionless pendulum"""
    KE_initial = calculate_kinetic_energy()
    PE_initial = calculate_potential_energy()
    E_initial = KE_initial + PE_initial
    
    # Simulate for several periods
    simulate(duration=10.0)
    
    KE_final = calculate_kinetic_energy()
    PE_final = calculate_potential_energy()
    E_final = KE_final + PE_final
    
    energy_drift = abs(E_final - E_initial) / E_initial
    assert energy_drift < 0.001
```

## Summary

✅ Physics engine configuration  
✅ Contact dynamics and friction  
✅ Realistic sensor simulation with noise  
✅ Performance optimization  
✅ Validation techniques  

## Practice Exercises

1. Configure contact sensors on robot feet
2. Add realistic IMU noise model
3. Simulate depth camera for obstacle detection
4. Validate pendulum physics

## Next Chapter

In **Chapter 9**, we'll explore:
- Unity for high-fidelity rendering
- VR/AR integration
- Human-robot interaction

👉 [Continue to Chapter 9 →](./chapter-9.md)
