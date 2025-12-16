---
sidebar_position: 4
---

import ChapterToolbar from '@site/src/components/ChapterToolbar';

<ChapterToolbar 
    chapterId="module-1/chapter-4" 
    chapterTitle="Launch Files and Parameter Management" 
/>

# Chapter 4: Launch Files and Parameter Management

## Introduction

As your robot systems grow in complexity, manually starting dozens of nodes becomes impractical. **Launch files** solve this by allowing you to start entire robot systems with a single command.

:::tip Learning Objectives
- Create Python launch files for multi-node systems
- Manage parameters at scale
- Use namespaces for multiple robots
- Implement event-driven launch flows
- Deploy complex robotic systems efficiently
:::

## Why Launch Files?

### The Problem

```bash
# Starting a humanoid robot manually (painful!)
ros2 run camera_driver camera_node &
ros2 run imu_driver imu_node &
ros2 run motor_control motor_controller &
ros2 run state_estimation ekf_node &
ros2 run planning mission_planner &
ros2 run control whole_body_controller &
# ... 20+ more nodes!
```

### The Solution

```bash
# Single command to start everything
ros2 launch humanoid_bringup robot.launch.py
```

## Python Launch Files

ROS 2 uses **Python** for launch files (more flexible than XML):

### Basic Launch File

**File:** `launch/simple_robot.launch.py`

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='motor_control',
            executable='motor_controller',
            name='motor_controller',
            output='screen'
        ),
        Node(
            package='sensor_fusion',
            executable='fusion_node',
            name='sensor_fusion',
            output='screen'
        ),
    ])
```

### Launch with Parameters

```python
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    # Declare arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )
    
    robot_name_arg = DeclareLaunchArgument(
        'robot_name',
        default_value='humanoid_01',
        description='Name of the robot'
    )
    
    # Use arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    robot_name = LaunchConfiguration('robot_name')
    
    return LaunchDescription([
        use_sim_time_arg,
        robot_name_arg,
        
        Node(
            package='motor_control',
            executable='motor_controller',
            name='motor_controller',
            parameters=[{
                'use_sim_time': use_sim_time,
                'robot_name': robot_name,
                'control_frequency': 100.0,
                'max_velocity': 2.0
            }]
        ),
    ])
```

**Usage:**
```bash
ros2 launch my_robot simple_robot.launch.py
ros2 launch my_robot simple_robot.launch.py use_sim_time:=true robot_name:=alpha
```

## Parameter Management

### Loading from YAML Files

**File:** `config/robot_params.yaml`

```yaml
motor_controller:
  ros__parameters:
    control_frequency: 100.0
    max_velocity: 2.0
    max_torque: 50.0
    joint_names: ["joint1", "joint2", "joint3"]

sensor_fusion:
  ros__parameters:
    imu_topic: "/imu/data"
    update_rate: 50.0
    filter_type: "ekf"
```

**Launch file:**

```python
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('my_robot'),
        'config',
        'robot_params.yaml'
    )
    
    return LaunchDescription([
        Node(
            package='motor_control',
            executable='motor_controller',
            parameters=[config]
        ),
        Node(
            package='sensor_fusion',
            executable='fusion_node',
            parameters=[config]
        ),
    ])
```

### Parameter Overrides

```python
Node(
    package='motor_control',
    executable='motor_controller',
    parameters=[
        config,  # Load from YAML
        {'control_frequency': 200.0}  # Override specific parameter
    ]
)
```

## Namespaces and Remapping

### Namespaces for Multiple Robots

```python
def generate_launch_description():
    return LaunchDescription([
        # Robot 1
        Node(
            package='motor_control',
            executable='motor_controller',
            namespace='robot1',
            name='motor_controller'
        ),
        # Robot 2
        Node(
            package='motor_control',
            executable='motor_controller',
            namespace='robot2',
            name='motor_controller'
        ),
    ])
```

**Resulting topics:**
```
/robot1/motor_controller/joint_states
/robot2/motor_controller/joint_states
```

### Topic Remapping

```python
Node(
    package='camera_driver',
    executable='camera_node',
    remappings=[
        ('/camera/image_raw', '/front_camera/image'),
        ('/camera/camera_info', '/front_camera/info')
    ]
)
```

## Advanced Launch Patterns

### Conditional Execution

```python
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    use_rviz_arg = DeclareLaunchArgument('use_rviz', default_value='true')
    use_rviz = LaunchConfiguration('use_rviz')
    
    return LaunchDescription([
        use_rviz_arg,
        
        # Only launch RViz if use_rviz is true
        Node(
            package='rviz2',
            executable='rviz2',
            condition=IfCondition(use_rviz),
            arguments=['-d', rviz_config]
        ),
        
        # Launch Gazebo only if NOT using real robot
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            condition=UnlessCondition(LaunchConfiguration('use_real_robot')),
            arguments=['-entity', 'humanoid', '-file', urdf_file]
        ),
    ])
```

### Including Other Launch Files

```python
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    sensors_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('sensor_drivers'),
                'launch',
                'sensors.launch.py'
            )
        ]),
        launch_arguments={'use_sim_time': 'false'}.items()
    )
    
    return LaunchDescription([sensors_launch])
```

### Event Handlers

```python
from launch.actions import RegisterEventHandler, LogInfo
from launch.event_handlers import OnProcessStart, OnProcessExit

def generate_launch_description():
    motor_node = Node(
        package='motor_control',
        executable='motor_controller',
        name='motor_controller'
    )
    
    planner_node = Node(
        package='planning',
        executable='planner',
        name='planner'
    )
    
    return LaunchDescription([
        motor_node,
        
        # Start planner only after motor controller starts
        RegisterEventHandler(
            OnProcessStart(
                target_action=motor_node,
                on_start=[
                    LogInfo(msg='Motor controller started, launching planner...'),
                    planner_node
                ]
            )
        ),
        
        # Handle motor controller exit
        RegisterEventHandler(
            OnProcessExit(
                target_action=motor_node,
                on_exit=[
                    LogInfo(msg='Motor controller exited, shutting down system...')
                ]
            )
        ),
    ])
```

## Complete Humanoid Launch Example

### Main Launch File

**File:** `launch/humanoid_bringup.launch.py`

```python
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction
)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Package directories
    pkg_humanoid = get_package_share_directory('humanoid_bringup')
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('use_rviz')
    robot_name = LaunchConfiguration('robot_name')
    
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )
    
    declare_use_rviz = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Launch RViz'
    )
    
    declare_robot_name = DeclareLaunchArgument(
        'robot_name',
        default_value='humanoid_alpha',
        description='Robot identifier'
    )
    
    # Configuration files
    motor_config = PathJoinSubstitution([
        FindPackageShare('humanoid_bringup'),
        'config',
        'motor_params.yaml'
    ])
    
    rviz_config = PathJoinSubstitution([
        FindPackageShare('humanoid_bringup'),
        'rviz',
        'humanoid.rviz'
    ])
    
    # Nodes
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': get_urdf_content()
        }]
    )
    
    motor_controller = Node(
        package='motor_control',
        executable='motor_controller',
        name='motor_controller',
        parameters=[motor_config, {'use_sim_time': use_sim_time}],
        output='screen'
    )
    
    sensor_fusion = Node(
        package='sensor_fusion',
        executable='fusion_node',
        name='sensor_fusion',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )
    
    mission_planner = TimerAction(
        period=3.0,  # Wait 3 seconds before starting
        actions=[
            Node(
                package='planning',
                executable='mission_planner',
                name='mission_planner',
                parameters=[{'use_sim_time': use_sim_time}]
            )
        ]
    )
    
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        condition=IfCondition(use_rviz)
    )
    
    return LaunchDescription([
        declare_use_sim_time,
        declare_use_rviz,
        declare_robot_name,
        robot_state_publisher,
        motor_controller,
        sensor_fusion,
        mission_planner,
        rviz,
    ])

def get_urdf_content():
    """Read URDF file content"""
    urdf_file = os.path.join(
        get_package_share_directory('humanoid_description'),
        'urdf',
        'humanoid.urdf'
    )
    with open(urdf_file, 'r') as f:
        return f.read()
```

### Simulation Launch File

**File:** `launch/simulation.launch.py`

```python
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    # Paths
    gazebo_pkg = FindPackageShare('gazebo_ros')
    world_file = os.path.join(
        get_package_share_directory('humanoid_gazebo'),
        'worlds',
        'humanoid_world.world'
    )
    
    # Start Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            gazebo_pkg, '/launch/gzserver.launch.py'
        ]),
        launch_arguments={'world': world_file}.items()
    )
    
    gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            gazebo_pkg, '/launch/gzclient.launch.py'
        ])
    )
    
    # Spawn robot
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'humanoid',
            '-file', get_urdf_path(),
            '-x', '0', '-y', '0', '-z', '0.5'
        ]
    )
    
    # Include main robot bringup
    robot_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('humanoid_bringup'),
            '/launch/humanoid_bringup.launch.py'
        ]),
        launch_arguments={'use_sim_time': 'true'}.items()
    )
    
    return LaunchDescription([
        gazebo,
        gazebo_client,
        spawn_robot,
        robot_bringup,
    ])
```

## Lifecycle Management

### Managed Nodes

```python
from launch_ros.actions import LifecycleNode
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition

def generate_launch_description():
    camera_node = LifecycleNode(
        package='camera_driver',
        executable='camera_node',
        name='camera',
        namespace='',
        output='screen'
    )
    
    # Configure the node on startup
    configure_camera = RegisterEventHandler(
        OnProcessStart(
            target_action=camera_node,
            on_start=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(camera_node),
                        transition_id=Transition.TRANSITION_CONFIGURE
                    )
                )
            ]
        )
    )
    
    # Activate after configuration
    activate_camera = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=camera_node,
            goal_state='inactive',
            entities=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(camera_node),
                        transition_id=Transition.TRANSITION_ACTIVATE
                    )
                )
            ]
        )
    )
    
    return LaunchDescription([
        camera_node,
        configure_camera,
        activate_camera,
    ])
```

## Node Composition

### Composable Nodes (Reduce Overhead)

```python
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    container = ComposableNodeContainer(
        name='sensor_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='camera_driver',
                plugin='camera::CameraNode',
                name='camera'
            ),
            ComposableNode(
                package='imu_driver',
                plugin='imu::ImuNode',
                name='imu'
            ),
            ComposableNode(
                package='sensor_fusion',
                plugin='fusion::FusionNode',
                name='fusion'
            ),
        ],
        output='screen',
    )
    
    return LaunchDescription([container])
```

**Benefits:**
- Reduced memory overhead
- Lower latency (intra-process communication)
- Better performance

## Testing Launch Files

```bash
# Check syntax
ros2 launch --show-args my_robot robot.launch.py

# Print launch tree
ros2 launch --print-description my_robot robot.launch.py

# Test with different arguments
ros2 launch my_robot robot.launch.py use_sim_time:=true use_rviz:=false
```

## Best Practices

### 1. Modular Launch Files

```
launch/
├── bringup.launch.py        # Main entry point
├── sensors.launch.py        # All sensors
├── control.launch.py        # Controllers
├── planning.launch.py       # Planning nodes
└── visualization.launch.py  # RViz, plots
```

### 2. Use Launch Arguments

```python
# ✅ DO: Provide defaults and descriptions
DeclareLaunchArgument(
    'control_frequency',
    default_value='100.0',
    description='Motor control loop frequency in Hz'
)

# ❌ DON'T: Hardcode values
control_frequency = 100.0  # No flexibility!
```

### 3. Organize Configuration

```
config/
├── simulation/
│   ├── motor_params.yaml
│   └── sensor_params.yaml
└── hardware/
    ├── motor_params.yaml
    └── sensor_params.yaml
```

## Summary

✅ Python launch files for complex systems  
✅ Parameter management with YAML files  
✅ Namespaces and remapping for multi-robot  
✅ Event-driven launch logic  
✅ Lifecycle node management  
✅ Node composition for performance  

## Practice Exercises

1. **Multi-Robot Launch**
   - Create launch file for 3 robots
   - Use namespaces to separate topics
   - Share common configuration

2. **Conditional Execution**
   - Launch simulation OR real hardware
   - Based on single argument
   - Include appropriate drivers

3. **Lifecycle Management**
   - Create managed camera node
   - Configure and activate in launch file
   - Handle shutdown gracefully

## Next Chapter

In **Chapter 5**, we'll learn:
- URDF (Unified Robot Description Format)
- Modeling humanoid robots
- Joints, links, and transforms
- Visualization in RViz

👉 [Continue to Chapter 5: URDF for Humanoid Robots →](./chapter-5.md)

---

:::tip Checkpoint
Ensure you can:
- Create Python launch files
- Load parameters from YAML
- Use namespaces for multiple robots
- Implement conditional execution
- Compose nodes for performance
:::
