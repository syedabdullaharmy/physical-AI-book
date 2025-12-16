---
sidebar_position: 3
---

# Chapter 3: Building ROS 2 Packages with Python

## Introduction

In this chapter, you'll master the art of creating production-ready ROS 2 packages. You'll learn organizational best practices, dependency management, and how to structure code for maintainability and reusability.

:::tip Learning Objectives
By the end of this chapter, you will:
- Create well-structured ROS 2 Python packages
- Manage dependencies effectively  
- Organize code for scalability
- Write reusable robot components
- Implement proper testing strategies
:::

## Package Structure

### anatomy of a Python Package

```
my_robot_pkg/
├── package.xml              # Package metadata and dependencies
├── setup.py                 # Python installation script
├── setup.cfg                # Install configuration
├── resource/               
│   └── my_robot_pkg         # Package marker file
├── my_robot_pkg/            # Python module (your code)
│   ├── __init__.py
│   ├── nodes/               # Node implementations
│   │   ├── __init__.py
│   │   ├── sensor_node.py
│   │   └── controller_node.py
│   ├── utils/               # Utility functions
│   │   ├── __init__.py
│   │   └── math_helpers.py
│   └── config/              # Configuration files
│       └── params.yaml
├── launch/                  # Launch files
│   ├── robot.launch.py
│   └── simulation.launch.py
├── test/                    # Unit tests
│   ├── test_sensor_node.py
│   └── test_controller.py
└── README.md                # Documentation
```

## Creating a Package

### Using `ros2 pkg create`

```bash
cd ~/ros2_ws/src

# Basic package
ros2 pkg create --build-type ament_python my_robot_pkg

# With dependencies
ros2 pkg create --build-type ament_python humanoid_control \
    --dependencies rclpy std_msgs geometry_msgs sensor_msgs

# With maintainer and license
ros2 pkg create --build-type ament_python advanced_robot \
    --dependencies rclpy \
    --maintainer-name "Your Name" \
    --maintainer-email "your.email@example.com" \
    --license Apache-2.0
```

### Package Metadata (`package.xml`)

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>humanoid_control</name>
  <version>1.0.0</version>
  <description>Control system for humanoid robots</description>
  <maintainer email="you@example.com">Your Name</maintainer>
  <license>Apache-2.0</license>

  <!-- Build dependencies -->
  <buildtool_depend>ament_python</buildtool_depend>

  <!-- Runtime dependencies -->
  <depend>rclpy</depend>
  <depend>std_msgs</depend>
  <depend>geometry_msgs</depend>
  <depend>sensor_msgs</depend>
  <depend>robot_interfaces</depend>

  <!-- Testing dependencies -->
  <test_depend>ament_copyright</test_depend>
  <test_depend>ament_flake8</test_depend>
  <test_depend>ament_pep257</test_depend>
  <test_depend>python3-pytest</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

### Setup Script (`setup.py`)

```python
from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'humanoid_control'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include launch files
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*.launch.py'))),
        # Include config files  
        (os.path.join('share', package_name, 'config'),
            glob(os.path.join('config', '*.yaml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='Humanoid robot control system',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sensor_node = humanoid_control.nodes.sensor_node:main',
            'controller_node = humanoid_control.nodes.controller_node:main',
            'planner_node = humanoid_control.nodes.planner_node:main',
        ],
    },
)
```

## Building a Complete Package

Let's build a complete package for a humanoid robot's motor control system:

### Step 1: Create Package Structure

```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python motor_control \
    --dependencies rclpy std_msgs sensor_msgs

cd motor_control

# Create directory structure
mkdir -p motor_control/{nodes,utils,config}
mkdir -p launch config test
```

### Step 2: Implement Motor Controller Node

**File:** `motor_control/nodes/motor_controller.py`

```python
#!/usr/bin/env python3
"""
Motor Controller Node for Humanoid Robot
Controls 17 motors (joints) based on commands
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
import numpy as np

class MotorController(Node):
    """
    Controls motor positions and velocities for humanoid joints
    """
    
    # Joint names for humanoid (17 DOF)
    JOINT_NAMES = [
        'head_pan', 'head_tilt',
        'left_shoulder_pitch', 'left_shoulder_roll', 'left_elbow',
        'right_shoulder_pitch', 'right_shoulder_roll', 'right_elbow',
        'waist_yaw', 'waist_pitch',
        'left_hip_pitch', 'left_hip_roll', 'left_knee', 'left_ankle',
        'right_hip_pitch', 'right_hip_roll', 'right_knee', 'right_ankle'
    ]
    
    def __init__(self):
        super().__init__('motor_controller')
        
        # Declare parameters
        self.declare_parameter('control_frequency', 100.0)  # Hz
        self.declare_parameter('max_velocity', 2.0)  # rad/s
        self.declare_parameter('max_torque', 50.0)  # Nm
        
        # Get parameters
        control_freq = self.get_parameter('control_frequency').value
        self.max_velocity = self.get_parameter('max_velocity').value
        self.max_torque = self.get_parameter('max_torque').value
        
        # Initialize state
        self.num_joints = len(self.JOINT_NAMES)
        self.current_positions = np.zeros(self.num_joints)
        self.current_velocities = np.zeros(self.num_joints)
        self.target_positions = np.zeros(self.num_joints)
        
        # Publishers
        self.joint_state_pub = self.create_publisher(
            JointState,
            '/joint_states',
            10
        )
        
        # Subscribers
        self.command_sub = self.create_subscription(
            Float64MultiArray,
            '/joint_commands',
            self.command_callback,
            10
        )
        
        # Control loop timer
        control_period = 1.0 / control_freq
        self.control_timer = self.create_timer(
            control_period,
            self.control_loop
        )
        
        self.get_logger().info(
            f'Motor controller initialized: {self.num_joints} joints, '
            f'{control_freq} Hz'
        )
    
    def command_callback(self, msg):
        """Process joint position commands"""
        if len(msg.data) != self.num_joints:
            self.get_logger().error(
                f'Invalid command size: expected {self.num_joints}, '
                f'got {len(msg.data)}'
            )
            return
        
        self.target_positions = np.array(msg.data)
        
    def control_loop(self):
        """Main control loop - executes at control frequency"""
        # Simple P-controller for position tracking
        position_error = self.target_positions - self.current_positions
        
        # Calculate desired velocities (proportional control)
        Kp = 5.0  # Proportional gain
        desired_velocities = Kp * position_error
        
        # Clamp velocities
        desired_velocities = np.clip(
            desired_velocities,
            -self.max_velocity,
            self.max_velocity
        )
        
        # Update positions (simplified dynamics)
        dt = 0.01  # Time step
        self.current_positions += desired_velocities * dt
        self.current_velocities = desired_velocities
        
        # Publish joint states
        self.publish_joint_state()
    
    def publish_joint_state(self):
        """Publish current joint states"""
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.JOINT_NAMES
        msg.position = self.current_positions.tolist()
        msg.velocity = self.current_velocities.tolist()
        msg.effort = [0.0] * self.num_joints  # Placeholder
        
        self.joint_state_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = MotorController()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Step 3: Create Utility Module

**File:** `motor_control/utils/kinematics.py`

```python
"""
Kinematic utilities for humanoid robots
"""

import numpy as np
from typing import Tuple, List

def forward_kinematics(joint_angles: np.ndarray,
                      link_lengths: List[float]) -> Tuple[float, float]:
    """
    Calculate end-effector position from joint angles
    
    Args:
        joint_angles: Array of joint angles in radians
        link_lengths: Lengths of robot links in meters
        
    Returns:
        (x, y) position of end effector
    """
    x = 0.0
    y = 0.0
    cumulative_angle = 0.0
    
    for angle, length in zip(joint_angles, link_lengths):
        cumulative_angle += angle
        x += length * np.cos(cumulative_angle)
        y += length * np.sin(cumulative_angle)
    
    return x, y

def inverse_kinematics_2dof(target_x: float,
                            target_y: float,
                            l1: float,
                            l2: float) -> Tuple[float, float]:
    """
    2-DOF inverse kinematics (analytical solution)
    
    Args:
        target_x: Target x position
        target_y: Target y position
        l1: Length of first link
        l2: Length of second link
        
    Returns:
        (theta1, theta2) joint angles in radians
    """
    # Distance to target
    d = np.sqrt(target_x**2 + target_y**2)
    
    # Check if target is reachable
    if d > (l1 + l2) or d < abs(l1 - l2):
        raise ValueError(f'Target ({target_x}, {target_y}) is unreachable')
    
    # Calculate joint angles using cosine law
    cos_theta2 = (d**2 - l1**2 - l2**2) / (2 * l1 * l2)
    theta2 = np.arccos(np.clip(cos_theta2, -1.0, 1.0))
    
    alpha = np.arctan2(target_y, target_x)
    beta = np.arctan2(l2 * np.sin(theta2), l1 + l2 * np.cos(theta2))
    theta1 = alpha - beta
    
    return theta1, theta2

def jacobian_2dof(theta1: float, theta2: float,
                 l1: float, l2: float) -> np.ndarray:
    """
    Calculate Jacobian matrix for 2-DOF arm
    
    Returns:
        2x2 Jacobian matrix
    """
    J = np.array([
        [-l1*np.sin(theta1) - l2*np.sin(theta1+theta2), -l2*np.sin(theta1+theta2)],
        [l1*np.cos(theta1) + l2*np.cos(theta1+theta2), l2*np.cos(theta1+theta2)]
    ])
    return J
```

### Step 4: Add Configuration File

**File:** `config/motor_params.yaml`

```yaml
motor_controller:
  ros__parameters:
    control_frequency: 100.0  # Hz
    max_velocity: 2.0  # rad/s
    max_torque: 50.0  # Nm
    
    # PID gains for each joint
    pid_gains:
      head:
        kp: 10.0
        ki: 0.1
        kd: 0.5
      arms:
        kp: 8.0
        ki: 0.05
        kd: 0.3
      legs:
        kp: 15.0
        ki: 0.2
        kd: 1.0
    
    # Joint limits (radians)
    joint_limits:
      head_pan: [-1.57, 1.57]
      head_tilt: [-0.79, 0.79]
      shoulder_pitch: [-2.09, 2.09]
      elbow: [0.0, 2.62]
      hip_pitch: [-1.57, 1.57]
      knee: [0.0, 2.62]
```

### Step 5: Update setup.py

```python
from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'motor_control'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*.launch.py'))),
        (os.path.join('share', package_name, 'config'),
            glob(os.path.join('config', '*.yaml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Robotics Team',
    maintainer_email='team@robot.com',
    description='Motor control package for humanoid robots',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'motor_controller = motor_control.nodes.motor_controller:main',
        ],
    },
)
```

## Testing Your Package

### Unit Tests

**File:** `test/test_kinematics.py`

```python
import unittest
import numpy as np
from motor_control.utils.kinematics import (
    forward_kinematics,
    inverse_kinematics_2dof
)

class TestKinematics(unittest.TestCase):
    
    def test_forward_kinematics(self):
        """Test forward kinematics calculation"""
        joint_angles = np.array([0.0, np.pi/4])
        link_lengths = [1.0, 1.0]
        
        x, y = forward_kinematics(joint_angles, link_lengths)
        
        # Expected position
        expected_x = 1.0 + np.cos(np.pi/4)
        expected_y = np.sin(np.pi/4)
        
        self.assertAlmostEqual(x, expected_x, places=5)
        self.assertAlmostEqual(y, expected_y, places=5)
    
    def test_inverse_kinematics(self):
        """Test inverse kinematics calculation"""
        target_x = 1.5
        target_y = 0.5
        l1 = 1.0
        l2 = 1.0
        
        theta1, theta2 = inverse_kinematics_2dof(target_x, target_y, l1, l2)
        
        # Verify by forward kinematics
        x, y = forward_kinematics(np.array([theta1, theta2]), [l1, l2])
        
        self.assertAlmostEqual(x, target_x, places=5)
        self.assertAlmostEqual(y, target_y, places=5)
    
    def test_unreachable_target(self):
        """Test that unreachable targets raise error"""
        with self.assertRaises(ValueError):
            inverse_kinematics_2dof(3.0, 0.0, 1.0, 1.0)

if __name__ == '__main__':
    unittest.main()
```

### Integration Tests

**File:** `test/test_motor_controller.py`

```python
import unittest
import rclpy
from rclpy.node import Node
from motor_control.nodes.motor_controller import MotorController
from std_msgs.msg import Float64MultiArray

class TestMotorController(unittest.TestCase):
    
    @classmethod
    def setUpClass(cls):
        rclpy.init()
    
    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()
    
    def test_node_creation(self):
        """Test that node can be created"""
        node = MotorController()
        self.assertIsNotNone(node)
        node.destroy_node()
    
    def test_command_processing(self):
        """Test processing of joint commands"""
        node = MotorController()
        
        # Create command message
        cmd = Float64MultiArray()
        cmd.data = [0.0] * node.num_joints
        cmd.data[0] = 0.5  # Command for first joint
        
        # Send command
        node.command_callback(cmd)
        
        # Verify target was updated
        self.assertEqual(node.target_positions[0], 0.5)
        
        node.destroy_node()

if __name__ == '__main__':
    unittest.main()
```

### Running Tests

```bash
# Run all tests
cd ~/ros2_ws
colcon test --packages-select motor_control

# View test results
colcon test-result --verbose

# Run specific test
python3 -m pytest src/motor_control/test/test_kinematics.py
```

## Building and Installing

```bash
cd ~/ros2_ws

# Build single package
colcon build --packages-select motor_control

# Build with verbose output
colcon build --packages-select motor_control --event-handlers console_direct+

# Build with compiler warnings
colcon build --packages-select motor_control --cmake-args -DCMAKE_BUILD_TYPE=Debug

# Clean build
rm -rf build/ install/ log/
colcon build --packages-select motor_control
```

## Running Your Package

```bash
# Source the workspace
source ~/ros2_ws/install/setup.bash

# Run the motor controller
ros2 run motor_control motor_controller

# Run with parameters
ros2 run motor_control motor_controller --ros-args \
    -p control_frequency:=50.0 \
    -p max_velocity:=1.5

# Run with config file
ros2 run motor_control motor_controller --ros-args \
    --params-file src/motor_control/config/motor_params.yaml
```

## Best Practices

### 1. Code Organization

```
✅ DO: Separate concerns
motor_control/
├── nodes/          # Node implementations
├── utils/          # Reusable utilities
├── interfaces/     # Custom messages (if needed)
└── config/         # Configuration files

❌ DON'T: Put everything in one file
motor_control/
└── everything.py   # 5000 lines of code!
```

### 2. Documentation

```python
"""
Motor Controller Package

This package provides motor control functionality for humanoid robots.

Nodes:
    - motor_controller: Main controller for 17-DOF humanoid
    
Topics:
    - /joint_commands (Float64MultiArray): Target joint positions
    - /joint_states (JointState): Current joint states
    
Parameters:
    - control_frequency (float): Control loop frequency in Hz
    - max_velocity (float): Maximum joint velocity in rad/s
"""
```

### 3. Error Handling

```python
def safe_control_loop(self):
    try:
        # Control logic
        self.update_motors()
    except Exception as e:
        self.get_logger().error(f'Control loop error: {e}')
        self.emergency_stop()
```

### 4. Logging Levels

```python
# Use appropriate logging levels
self.get_logger().debug('Detailed debug info')
self.get_logger().info('Normal operation info')
self.get_logger().warn('Warning about potential issues')
self.get_logger().error('Error that affects functionality')
self.get_logger().fatal('Critical error, shutting down')
```

## Dependencies Management

### Adding External Python Packages

**package.xml:**
```xml
<depend>python3-numpy</depend>
<depend>python3-scipy</depend>
```

**setup.py:**
```python
install_requires=[
    'setuptools',
    'numpy',
    'scipy',
    'matplotlib',
],
```

### Installing System Dependencies

```bash
# Use rosdep to install all dependencies
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
```

## Summary

In this chapter, you learned:

✅ How to structure ROS 2 Python packages  
✅ Package metadata and dependency management  
✅ Creating reusable utility modules  
✅ Writing comprehensive tests  
✅ Building and installing packages  
✅ Best practices for code organization  

## Practice Exercises

1. **Package Creation**
   - Create a `sensor_fusion` package
   - Implement IMU and odometry fusion
   - Add configuration files for sensor parameters

2. **Testing Suite**
   - Write unit tests for all utility functions
   - Add integration tests for node communication
   - Achieve >80% code coverage

3. **Refactoring Challenge**
   - Take an existing monolithic node
   - Split into modular package structure
   - Add proper documentation

## Next Chapter

In **Chapter 4**, we'll explore:
- Launch files for complex robot systems
- Multi-node coordination
- Parameter management at scale
- Namespace and remapping

👉 [Continue to Chapter 4: Launch Files →](./chapter-4.md)

---

:::tip Checkpoint
Before moving on, ensure you can:
- Create a well-structured ROS 2 package
- Write and run unit tests
- Manage dependencies properly
- Build and install packages
- Organize code for maintainability
:::
