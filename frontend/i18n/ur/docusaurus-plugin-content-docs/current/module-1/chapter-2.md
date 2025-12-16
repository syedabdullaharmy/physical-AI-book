---
sidebar_position: 2
---

# Chapter 2: Nodes, Topics, and Services

## Introduction

In this chapter, we'll dive deep into the fundamental building blocks of ROS 2: **Nodes**, **Topics**, and **Services**. You'll learn how to create robust, production-ready nodes that communicate efficiently.

:::tip Learning Objectives
By the end of this chapter, you will:
- Master node lifecycle and management
- Create custom message types
- Implement reliable topic communication
- Build request-response services
- Apply best practices for distributed systems
:::

## Deep Dive: Nodes

### What Makes a Good Node?

A well-designed ROS 2 node follows the **Single Responsibility Principle**:

```python
# ✅ GOOD: Focused, single-purpose nodes
class CameraNode(Node):
    """Captures and publishes camera frames"""
    
class ImageProcessorNode(Node):
    """Processes images and detects objects"""
    
class MotionPlannerNode(Node):
    """Plans robot trajectories"""

# ❌ BAD: Monolithic "do everything" node
class RobotControllerNode(Node):
    """Handles camera, planning, control, UI, logging..."""
    # Too many responsibilities!
```

### Node Lifecycle

ROS 2 nodes can be **managed** or **unmanaged**:

#### Unmanaged Nodes (Simple)

Most nodes you create are unmanaged—they start immediately:

```python
class SimpleNode(Node):
    def __init__(self):
        super().__init__('simple_node')
        self.get_logger().info('Node started!')
        # Node is active immediately
```

#### Managed Nodes (Lifecycle)

For production systems, use **lifecycle nodes** for controlled startup:

```python
from rclpy.lifecycle import LifecycleNode, LifecycleState, TransitionCallbackReturn

class ManagedCameraNode(LifecycleNode):
    def __init__(self):
        super().__init__('camera_node')
        
    def on_configure(self, state: LifecycleState):
        """Called when transitioning to 'configured' state"""
        self.get_logger().info('Configuring camera...')
        self.camera = self.initialize_camera()
        return TransitionCallbackReturn.SUCCESS
    
    def on_activate(self, state: LifecycleState):
        """Called when transitioning to 'active' state"""
        self.get_logger().info('Activating camera...')
        self.camera.start_capture()
        return TransitionCallbackReturn.SUCCESS
    
    def on_deactivate(self, state: LifecycleState):
        """Called when transitioning to 'inactive' state"""
        self.get_logger().info('Deactivating camera...')
        self.camera.stop_capture()
        return TransitionCallbackReturn.SUCCESS
    
    def on_cleanup(self, state: LifecycleState):
        """Called when cleaning up"""
        self.get_logger().info('Cleaning up camera resources...')
        self.camera.release()
        return TransitionCallbackReturn.SUCCESS
```

**Lifecycle State Machine:**

```
    configuring
        ↓
   [configured]
        ↓
    activating
        ↓
    [active] ←─────────────┐
        ↓                  │
   deactivating            │
        ↓                  │
   [inactive] ─────────────┘
        ↓
   cleaning up
        ↓
   [finalized]
```

### Node Components

#### 1. Timers

Execute callbacks at regular intervals:

```python
class TimerNode(Node):
    def __init__(self):
        super().__init__('timer_node')
        
        # Timer fires every 0.5 seconds
        self.timer = self.create_timer(0.5, self.timer_callback)
        self.count = 0
    
    def timer_callback(self):
        self.get_logger().info(f'Timer tick: {self.count}')
        self.count += 1
```

**Use cases:**
- Sensor polling
- Periodic health checks
- Regular state updates

#### 2. Parameters

Dynamic configuration:

```python
class ParameterizedNode(Node):
    def __init__(self):
        super().__init__('param_node')
        
        # Declare parameters with defaults and constraints
        self.declare_parameter('max_speed', 1.0, 
            ParameterDescriptor(
                description='Maximum robot speed (m/s)',
                type=ParameterType.PARAMETER_DOUBLE,
                floating_point_range=[FloatingPointRange(
                    from_value=0.0,
                    to_value=5.0,
                    step=0.1
                )]
            )
        )
        
        self.declare_parameter('robot_name', 'humanoid_01')
        self.declare_parameter('use_lidar', True)
        
        # Add parameter callback for dynamic updates
        self.add_on_set_parameters_callback(self.parameter_callback)
        
    def parameter_callback(self, params):
        """Called when parameters are updated"""
        for param in params:
            if param.name == 'max_speed':
                self.max_speed = param.value
                self.get_logger().info(f'Max speed updated to: {param.value}')
        return SetParametersResult(successful=True)
```

**Set parameters at runtime:**

```bash
# Command line
ros2 param set /param_node max_speed 2.5

# From code
node.set_parameters([Parameter('max_speed', value=2.0)])

# From YAML file
ros2 run my_package param_node --ros-args --params-file config.yaml
```

**config.yaml:**
```yaml
param_node:
  ros__parameters:
    max_speed: 2.0
    robot_name: "humanoid_02"
    use_lidar: true
```

## Custom Message Types

### Why Custom Messages?

Standard messages (`std_msgs`, `geometry_msgs`) are useful, but you'll need custom messages for:

- Domain-specific data structures
- Complex robot states
- Sensor fusion results
- Application-specific commands

### Creating a Custom Message

**Step 1: Create package structure**

```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_cmake robot_interfaces
cd robot_interfaces
mkdir msg
```

**Step 2: Define message file**

File: `msg/RobotStatus.msg`

```
# Custom message for robot status

# Header with timestamp
std_msgs/Header header

# Robot identification
string robot_name
uint8 robot_id

# Health status
bool is_operational
float32 battery_level        # 0.0 to 1.0
float32 temperature          # Celsius

# Position (x, y, z)
geometry_msgs/Point position

# Velocity (linear and angular)
geometry_msgs/Twist velocity

# Error codes
uint8 ERROR_NONE=0
uint8 ERROR_LOW_BATTERY=1
uint8 ERROR_OVERHEAT=2
uint8 ERROR_COLLISION=3
uint8 error_code
```

**Step 3: Update CMakeLists.txt**

```cmake
cmake_minimum_required(VERSION 3.8)
project(robot_interfaces)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# Generate messages
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/RobotStatus.msg"
  DEPENDENCIES std_msgs geometry_msgs
)

ament_package()
```

**Step 4: Update package.xml**

```xml
<build_depend>rosidl_default_generators</build_depend>
<exec_depend>rosidl_default_runtime</exec_depend>
<member_of_group>rosidl_interface_packages</member_of_group>

<depend>std_msgs</depend>
<depend>geometry_msgs</depend>
```

**Step 5: Build**

```bash
cd ~/ros2_ws
colcon build --packages-select robot_interfaces
source install/setup.bash
```

### Using Custom Messages

```python
from robot_interfaces.msg import RobotStatus
from geometry_msgs.msg import Point, Twist

class StatusPublisher(Node):
    def __init__(self):
        super().__init__('status_publisher')
        self.publisher = self.create_publisher(RobotStatus, 'robot_status', 10)
        self.timer = self.create_timer(1.0, self.publish_status)
        
    def publish_status(self):
        msg = RobotStatus()
        
        # Header
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        
        # Robot info
        msg.robot_name = 'Humanoid Alpha'
        msg.robot_id = 1
        
        # Health
        msg.is_operational = True
        msg.battery_level = 0.85
        msg.temperature = 42.5
        
        # Position
        msg.position.x = 1.5
        msg.position.y = 2.0
        msg.position.z = 0.0
        
        # Velocity
        msg.velocity.linear.x = 0.5
        msg.velocity.angular.z = 0.1
        
        # Error code
        msg.error_code = RobotStatus.ERROR_NONE
        
        self.publisher.publish(msg)
        self.get_logger().info(f'Published status: battery={msg.battery_level:.2f}')
```

## Topics: Advanced Patterns

### Publisher-Subscriber Pattern

#### Basic Publisher (Review)

```python
class SensorPublisher(Node):
    def __init__(self):
        super().__init__('sensor_publisher')
        self.publisher = self.create_publisher(
            Temperature,  # Message type
            'temperature', # Topic name
            10            # Queue size
        )
        self.timer = self.create_timer(0.1, self.publish_data)
        
    def publish_data(self):
        msg = Temperature()
        msg.temperature = self.read_sensor()
        self.publisher.publish(msg)
```

#### Basic Subscriber (Review)

```python
class SensorSubscriber(Node):
    def __init__(self):
        super().__init__('sensor_subscriber')
        self.subscription = self.create_subscription(
            Temperature,
            'temperature',
            self.listener_callback,
            10
        )
        
    def listener_callback(self, msg):
        self.get_logger().info(f'Temperature: {msg.temperature}°C')
```

### Quality of Service (QoS) Profiles

QoS policies determine message delivery behavior:

```python
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

# Sensor data: Best effort, can drop old messages
sensor_qos = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=10,
    durability=QoSDurabilityPolicy.VOLATILE
)

# Commands: Must be reliable
command_qos = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    history=QoSHistoryPolicy.KEEP_ALL,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL
)

# Critical state: Need latest + history for late joiners
state_qos = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL
)

# Use in publisher/subscriber
self.publisher = self.create_publisher(String, '/topic', sensor_qos)
self.subscription = self.create_subscription(String, '/topic', callback, sensor_qos)
```

**QoS Policy Options:**

| Policy | Options | Use Case |
|--------|---------|----------|
| **Reliability** | `BEST_EFFORT`, `RELIABLE` | Sensor data vs. Commands |
| **Durability** | `VOLATILE`, `TRANSIENT_LOCAL` | Real-time vs. Late joiners |
| **History** | `KEEP_LAST`, `KEEP_ALL` | Buffer size strategy |
| **Depth** | Integer | Queue size |

### Message Filtering

Subscribe only to messages matching criteria:

```python
class FilteredSubscriber(Node):
    def __init__(self):
        super().__init__('filtered_subscriber')
        
        self.subscription = self.create_subscription(
            RobotStatus,
            'robot_status',
            self.status_callback,
            10
        )
        
    def status_callback(self, msg):
        # Filter: Only process high-priority robots
        if msg.robot_id < 10:
            return
        
        # Filter: Only process errors
        if msg.error_code == RobotStatus.ERROR_NONE:
            return
            
        # Process filtered message
        self.handle_error(msg)
```

## Services: Request-Response Communication

### When to Use Services

Use **services** for:
- ✅ One-shot requests (not continuous data)
- ✅ Synchronous operations (wait for response)
- ✅ Actions that complete quickly (< 1 second)
- ✅ Configuration changes

Use **topics** for:
- ✅ Continuous data streams
- ✅ Many subscribers
- ✅ Fire-and-forget

### Creating Custom Service

**Step 1: Define service**

File: `srv/SetRobotMode.srv`

```
# Request
string mode      # "idle", "autonomous", "manual"
bool emergency_stop

---

# Response
bool success
string message
float32 transition_time  # seconds
```

**Step 2: Update CMakeLists.txt**

```cmake
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/RobotStatus.msg"
  "srv/SetRobotMode.srv"
  DEPENDENCIES std_msgs geometry_msgs
)
```

**Step 3: Implement Service Server**

```python
from robot_interfaces.srv import SetRobotMode

class RobotModeServer(Node):
    def __init__(self):
        super().__init__('robot_mode_server')
        
        self.srv = self.create_service(
            SetRobotMode,
            'set_robot_mode',
            self.set_mode_callback
        )
        
        self.current_mode = 'idle'
        self.get_logger().info('Robot mode service ready')
    
    def set_mode_callback(self, request, response):
        """Handle mode change requests"""
        self.get_logger().info(
            f'Received request: mode={request.mode}, '
            f'emergency_stop={request.emergency_stop}'
        )
        
        # Validate requested mode
        valid_modes = ['idle', 'autonomous', 'manual']
        if request.mode not in valid_modes:
            response.success = False
            response.message = f'Invalid mode: {request.mode}'
            response.transition_time = 0.0
            return response
        
        # Handle emergency stop
        if request.emergency_stop:
            self.emergency_stop_robot()
            self.current_mode = 'idle'
            response.success = True
            response.message = 'Emergency stop activated'
            response.transition_time = 0.1
            return response
        
        # Perform mode transition
        old_mode = self.current_mode
        transition_time = self.transition_to_mode(request.mode)
        self.current_mode = request.mode
        
        response.success = True
        response.message = f'Transitioned from {old_mode} to {request.mode}'
        response.transition_time = transition_time
        
        self.get_logger().info(response.message)
        return response
    
    def transition_to_mode(self, mode):
        """Simulate mode transition"""
        import time
        time.sleep(0.5)  # Simulate transition delay
        return 0.5
    
    def emergency_stop_robot(self):
        """Emergency stop procedure"""
        self.get_logger().warn('EMERGENCY STOP ACTIVATED!')
        # Stop all motors, etc.
```

**Step 4: Implement Service Client**

```python
class RobotModeClient(Node):
    def __init__(self):
        super().__init__('robot_mode_client')
        
        self.client = self.create_client(SetRobotMode, 'set_robot_mode')
        
        # Wait for service to be available
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for service...')
    
    def send_request(self, mode, emergency_stop=False):
        """Send mode change request"""
        request = SetRobotMode.Request()
        request.mode = mode
        request.emergency_stop = emergency_stop
        
        # Asynchronous call
        future = self.client.call_async(request)
        future.add_done_callback(self.response_callback)
    
    def response_callback(self, future):
        """Handle service response"""
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(
                    f'✓ {response.message} '
                    f'(took {response.transition_time:.2f}s)'
                )
            else:
                self.get_logger().error(f'✗ {response.message}')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')

# Usage
def main():
    rclpy.init()
    client = RobotModeClient()
    
    # Change to autonomous mode
    client.send_request('autonomous')
    
    rclpy.spin_once(client, timeout_sec=5.0)
    client.destroy_node()
    rclpy.shutdown()
```

### Synchronous vs. Asynchronous Service Calls

#### Synchronous (Blocking)

```python
# Blocks until response received
request = SetRobotMode.Request()
request.mode = 'autonomous'

try:
    response = self.client.call(request, timeout_sec=5.0)
    print(f'Success: {response.success}')
except Exception as e:
    print(f'Service call failed: {e}')
```

#### Asynchronous (Non-blocking)

```python
# Returns immediately, callback called when response arrives
future = self.client.call_async(request)
future.add_done_callback(self.handle_response)

# Continue with other work...
```

## Real-World Example: Humanoid Robot Controller

Let's build a complete system with multiple nodes:

### Architecture

```
┌─────────────┐      ┌──────────────┐      ┌─────────────┐
│   Sensor    │─────▶│ State        │─────▶│   Motor     │
│   Fusion    │      │ Estimator    │      │ Controller  │
└─────────────┘      └──────────────┘      └─────────────┘
       │                     │                      │
       │                     ▼                      │
       │              ┌─────────────┐               │
       └─────────────▶│   Mission   │◀──────────────┘
                      │   Planner   │
                      └─────────────┘
                            │
                            ▼
                     [Services for]
                     [mode changes]
```

### 1. Sensor Fusion Node

```python
from sensor_msgs.msg import Imu, JointState
from robot_interfaces.msg import RobotStatus

class SensorFusion(Node):
    def __init__(self):
        super().__init__('sensor_fusion')
        
        # Subscribers
        self.imu_sub = self.create_subscription(Imu, '/imu/data', self.imu_callback, 10)
        self.joint_sub = self.create_subscription(JointState, '/joint_states', self.joint_callback, 10)
        
        # Publisher
        self.status_pub = self.create_publisher(RobotStatus, '/robot_status', 10)
        
        # State
        self.imu_data = None
        self.joint_data = None
        
        # Timer for fusion
        self.timer = self.create_timer(0.05, self.publish_fused_state)  # 20 Hz
    
    def imu_callback(self, msg):
        self.imu_data = msg
    
    def joint_callback(self, msg):
        self.joint_data = msg
    
    def publish_fused_state(self):
        if self.imu_data is None or self.joint_data is None:
            return
        
        status = RobotStatus()
        status.header.stamp = self.get_clock().now().to_msg()
        
        # Fuse sensor data
        # ... (simplified)
        
        self.status_pub.publish(status)
```

### 2. Mission Planner Node

```python
class MissionPlanner(Node):
    def __init__(self):
        super().__init__('mission_planner')
        
        # Subscribe to robot status
        self.status_sub = self.create_subscription(
            RobotStatus,
            '/robot_status',
            self.status_callback,
            10
        )
        
        # Service for mode changes
        self.mode_client = self.create_client(SetRobotMode, '/set_robot_mode')
        
        # Current state
        self.robot_status = None
    
    def status_callback(self, msg):
        self.robot_status = msg
        
        # Make decisions based on status
        if msg.battery_level < 0.2:
            self.get_logger().warn('Low battery! Returning to charge station')
            self.request_mode_change('idle')
        
        if msg.error_code != RobotStatus.ERROR_NONE:
            self.handle_error(msg.error_code)
    
    def request_mode_change(self, mode):
        request = SetRobotMode.Request()
        request.mode = mode
        future = self.mode_client.call_async(request)
```

## Debugging and Introspection

### Command-Line Tools

```bash
# List all nodes
ros2 node list

# Detailed node info
ros2 node info /sensor_fusion

# Topic list
ros2 topic list -t  # with types

# Topic info (publishers/subscribers)
ros2 topic info /robot_status

# Echo messages
ros2 topic echo /robot_status

# Publish from CLI
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.5}, angular: {z: 0.1}}"

# Service list
ros2 service list -t

# Call service from CLI
ros2 service call /set_robot_mode robot_interfaces/srv/SetRobotMode \
  "{mode: 'autonomous', emergency_stop: false}"

# Monitor topic rate
ros2 topic hz /robot_status

# Monitor topic bandwidth
ros2 topic bw /camera/image_raw
```

### Visualization with rqt

```bash
# Node graph
rqt_graph

# Topic monitor
rqt_topic

# Service caller
rqt_service_caller

# Message publisher
rqt_publisher

# All-in-one
rqt
```

## Best Practices

### 1. Error Handling

```python
class RobustNode(Node):
    def __init__(self):
        super().__init__('robust_node')
        
    def subscriber_callback(self, msg):
        try:
            result = self.process_message(msg)
            self.publish_result(result)
        except ValueError as e:
            self.get_logger().error(f'Invalid message data: {e}')
        except Exception as e:
            self.get_logger().error(f'Unexpected error: {e}')
            # Don't crash the node!
```

### 2. Resource Cleanup

```python
class CleanNode(Node):
    def __init__(self):
        super().__init__('clean_node')
        self.file_handle = open('data.txt', 'w')
    
    def __del__(self):
        """Cleanup resources"""
        if hasattr(self, 'file_handle'):
            self.file_handle.close()

# Or use context managers
with CleanNode() as node:
    rclpy.spin(node)
```

### 3. Testing

```python
import unittest
from rclpy.node import Node

class TestSensorFusion(unittest.TestCase):
    def test_fusion_output(self):
        node = SensorFusion()
        
        # Simulate sensor input
        imu_msg = create_test_imu()
        node.imu_callback(imu_msg)
        
        # Verify output
        self.assertIsNotNone(node.imu_data)
```

## Summary

In this chapter, you learned:

✅ Node lifecycle management  
✅ Creating custom messages and services  
✅ Advanced topic communication patterns  
✅ QoS profiles for reliable communication  
✅ Request-response patterns with services  
✅ Real-world multi-node system design  
✅ Debugging and introspection tools  

## Practice Exercises

1. **Custom Message Challenge**
   - Create a `HumanoidJointState` message with 17 joints
   - Include position, velocity, and effort for each
   - Publish at 100 Hz

2. **Service Implementation**
   - Create a `CalculateTrajectory` service
   - Input: start and goal positions
   - Output: waypoint list and estimated time

3. **Multi-Node System**
   - Build a 3-node system:
     - Sensor simulator (publishes fake IMU data)
     - Filter node (Kalman filter)
     - Logger node (saves filtered data)

## Additional Resources

- [ROS 2 Messages Documentation](https://docs.ros.org/en/humble/Concepts/About-ROS-Interfaces.html)
- [QoS Documentation](https://docs.ros.org/en/humble/Concepts/About-Quality-of-Service-Settings.html)
- [rqt Tutorial](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Introducing-Turtlesim/Introducing-Turtlesim.html#rqt)

## Next Chapter

Ready for more? In **Chapter 3**, we'll explore:
- Building ROS 2 packages with Python
- Package structure and organization
- Dependencies and build systems
- Launch files for multi-node systems

👉 [Continue to Chapter 3: Building ROS 2 Packages →](./chapter-3.md)

---

:::tip Checkpoint
Before moving on, ensure you can:
- Create custom messages and services
- Implement lifecycle nodes
- Use QoS profiles appropriately
- Debug with ros2 CLI tools
- Design multi-node systems
:::
