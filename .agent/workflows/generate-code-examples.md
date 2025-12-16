---
description: Generate tested, production-quality code examples for robotics topics
---

# Agent Skill: Generate Code Examples for Robotics

This workflow creates high-quality, tested code examples for the Physical AI & Humanoid Robotics textbook.

## Input Requirements
- **Topic**: The specific robotics concept (e.g., "ROS 2 Publisher", "URDF Model")
- **Language**: Python, C++, YAML, XML, etc.
- **Complexity Level**: Beginner, Intermediate, or Advanced
- **Framework**: ROS 2, Gazebo, Isaac Sim, etc.
- **Purpose**: What the code demonstrates

## Code Quality Standards

### 1. Structure
```python
"""
Module docstring explaining purpose and usage.

Example:
    $ ros2 run my_package my_node

Author: Physical AI Course
License: MIT
"""

# Standard library imports
import sys
from typing import List, Optional

# Third-party imports
import rclpy
from rclpy.node import Node

# Local imports
from my_package.utils import helper_function


class MyRobotNode(Node):
    """
    Clear class docstring explaining functionality.
    
    Attributes:
        attribute_name: Description of attribute
    """
    
    def __init__(self):
        """Initialize the node with parameters and publishers."""
        super().__init__('my_robot_node')
        # Implementation
```

### 2. Documentation
- **Module docstring**: Purpose, usage example, author
- **Class docstring**: Functionality, attributes, methods
- **Method docstring**: Parameters, returns, raises
- **Inline comments**: Explain complex logic
- **Type hints**: For all function parameters and returns

### 3. Error Handling
```python
def safe_operation(self, value: float) -> Optional[float]:
    """
    Perform operation with proper error handling.
    
    Args:
        value: Input value to process
        
    Returns:
        Processed value or None if error
        
    Raises:
        ValueError: If value is out of range
    """
    try:
        if value < 0:
            raise ValueError(f"Value must be positive, got {value}")
        
        result = self.process(value)
        return result
        
    except ValueError as e:
        self.get_logger().error(f"Invalid value: {e}")
        return None
    except Exception as e:
        self.get_logger().error(f"Unexpected error: {e}")
        return None
```

### 4. ROS 2 Best Practices
- Use lifecycle nodes when appropriate
- Implement proper cleanup in destroy methods
- Use parameters for configuration
- Add QoS profiles for reliability
- Include launch files for complex setups

## Example Templates

### ROS 2 Publisher Node
```python
"""
Simple ROS 2 publisher node demonstrating basic pub/sub pattern.

This node publishes sensor data at a fixed rate.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import JointState


class SensorPublisher(Node):
    """Publishes simulated sensor data."""
    
    def __init__(self):
        """Initialize publisher and timer."""
        super().__init__('sensor_publisher')
        
        # Declare parameters
        self.declare_parameter('publish_rate', 10.0)
        rate = self.get_parameter('publish_rate').value
        
        # Create publisher
        self.publisher = self.create_publisher(
            JointState,
            'joint_states',
            10  # QoS history depth
        )
        
        # Create timer
        self.timer = self.create_timer(
            1.0 / rate,
            self.timer_callback
        )
        
        self.get_logger().info(f'Publisher started at {rate} Hz')
    
    def timer_callback(self):
        """Publish sensor data."""
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['joint1', 'joint2']
        msg.position = [0.0, 1.57]
        
        self.publisher.publish(msg)
        self.get_logger().debug('Published joint states')


def main(args=None):
    """Main entry point."""
    rclpy.init(args=args)
    node = SensorPublisher()
    
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

### ROS 2 Service Server
```python
"""
ROS 2 service server for robot control commands.
"""

import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts


class ControlService(Node):
    """Service server for robot control."""
    
    def __init__(self):
        """Initialize service server."""
        super().__init__('control_service')
        
        self.srv = self.create_service(
            AddTwoInts,
            'control_robot',
            self.handle_control_request
        )
        
        self.get_logger().info('Control service ready')
    
    def handle_control_request(self, request, response):
        """
        Handle incoming control requests.
        
        Args:
            request: Service request
            response: Service response
            
        Returns:
            Populated response
        """
        self.get_logger().info(
            f'Received request: {request.a} + {request.b}'
        )
        
        response.sum = request.a + request.b
        return response


def main(args=None):
    """Main entry point."""
    rclpy.init(args=args)
    node = ControlService()
    
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

### URDF Robot Model
```xml
<?xml version="1.0"?>
<!-- Simple humanoid robot URDF model -->
<robot name="simple_humanoid">
  
  <!-- Base Link -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.3 0.2 0.4"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 0.8 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.3 0.2 0.4"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="10.0"/>
      <inertia ixx="0.1" ixy="0.0" ixz="0.0"
               iyy="0.1" iyz="0.0" izz="0.1"/>
    </inertial>
  </link>
  
  <!-- Right Arm Joint -->
  <joint name="right_shoulder" type="revolute">
    <parent link="base_link"/>
    <child link="right_arm"/>
    <origin xyz="0.15 0 0.15" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="100" velocity="1.0"/>
  </joint>
  
  <!-- Right Arm Link -->
  <link name="right_arm">
    <visual>
      <geometry>
        <cylinder length="0.4" radius="0.05"/>
      </geometry>
      <origin xyz="0 0 -0.2" rpy="0 0 0"/>
      <material name="grey">
        <color rgba="0.5 0.5 0.5 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.4" radius="0.05"/>
      </geometry>
      <origin xyz="0 0 -0.2" rpy="0 0 0"/>
    </collision>
    <inertial>
      <mass value="2.0"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0"
               iyy="0.01" iyz="0.0" izz="0.01"/>
    </inertial>
  </link>
  
</robot>
```

### Launch File
```python
"""
Launch file for robot simulation.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Generate launch description."""
    
    # Declare arguments
    use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )
    
    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'robot_description': 'path/to/urdf'
        }]
    )
    
    # Gazebo
    gazebo = IncludeLaunchDescription(
        PathJoinSubstitution([
            FindPackageShare('gazebo_ros'),
            'launch',
            'gazebo.launch.py'
        ])
    )
    
    return LaunchDescription([
        use_sim_time,
        robot_state_publisher,
        gazebo
    ])
```

## Execution Steps

1. **Understand Requirements**
   - Clarify the concept to demonstrate
   - Identify target audience level
   - Determine framework/library versions

2. **Design Code Structure**
   - Plan class/function organization
   - Identify key components
   - Design interfaces

3. **Implement Code**
   - Write clean, documented code
   - Follow style guides (PEP 8 for Python)
   - Add comprehensive comments
   - Include error handling

4. **Test Code**
   - Create test environment
   - Run code and verify output
   - Test edge cases
   - Document expected behavior

5. **Create Supporting Files**
   - package.xml for ROS 2 packages
   - setup.py for Python packages
   - CMakeLists.txt for C++ packages
   - README with usage instructions

6. **Document Usage**
   - Write clear usage instructions
   - Include example commands
   - Show expected output
   - List dependencies

## File Organization
```
frontend/docs/module-X/code/chapter-Y/
├── README.md                 # Usage instructions
├── package.xml              # ROS 2 package manifest
├── setup.py                 # Python package setup
├── my_package/
│   ├── __init__.py
│   ├── my_node.py          # Main node implementation
│   └── utils.py            # Helper functions
├── launch/
│   └── example.launch.py   # Launch file
├── config/
│   └── params.yaml         # Parameters
└── test/
    └── test_my_node.py     # Unit tests
```

## Success Criteria
- [ ] Code runs without errors
- [ ] All functions have docstrings
- [ ] Includes error handling
- [ ] Follows style guide
- [ ] Has usage examples
- [ ] Tested in target environment
- [ ] Dependencies documented
- [ ] Comments explain complex logic
