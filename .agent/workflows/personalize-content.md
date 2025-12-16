---
description: Personalize chapter content based on user background and experience
---

# Agent Skill: Personalize Content

This workflow adapts chapter content to match the user's software and hardware background, creating a customized learning experience.

## Input Requirements
- **Chapter Content**: The original chapter markdown
- **User Profile**: User's background information
  - Software experience level (Beginner/Intermediate/Advanced)
  - Hardware experience level (Beginner/Intermediate/Advanced)
  - Programming languages known
  - Previous robotics experience
  - Specific interests/goals

## Personalization Strategies

### 1. For Software Beginners
- Add more detailed code explanations
- Include programming fundamentals refreshers
- Provide step-by-step breakdowns
- Use analogies to everyday concepts
- Include "What is..." sidebars for technical terms

**Example Transformation:**
```markdown
<!-- Original -->
Create a publisher using `create_publisher()` method.

<!-- Personalized for Beginners -->
**What is a Publisher?**  
A publisher is like a radio station that broadcasts messages. Any node that's "tuned in" (subscribed) can receive these messages.

To create a publisher in ROS 2, we use the `create_publisher()` method:

\`\`\`python
# This creates a publisher that will broadcast messages
self.publisher = self.create_publisher(
    String,        # Type of message (like choosing AM or FM)
    'my_topic',    # Channel name (like 101.5 FM)
    10             # How many messages to remember
)
\`\`\`

**Breaking it down:**
- `String`: The type of data we're sending (text in this case)
- `'my_topic'`: The name of our broadcast channel
- `10`: Buffer size - how many messages to keep if subscribers are slow
```

### 2. For Software Intermediates
- Focus on best practices
- Include performance considerations
- Show alternative approaches
- Discuss design patterns
- Add debugging tips

**Example:**
```markdown
<!-- Personalized for Intermediates -->
**Publisher Implementation Best Practices:**

\`\`\`python
class SensorPublisher(Node):
    def __init__(self):
        super().__init__('sensor_publisher')
        
        # Use QoS profiles for reliability
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        self.publisher = self.create_publisher(
            JointState,
            'joint_states',
            qos_profile
        )
\`\`\`

**Design Considerations:**
- **QoS Profiles**: Match publisher/subscriber QoS for communication
- **Topic Naming**: Use descriptive, hierarchical names (`/robot/sensors/joint_states`)
- **Message Types**: Choose appropriate message types from standard libraries
- **Publishing Rate**: Balance between data freshness and network load

**Common Pitfalls:**
- Publishing before subscribers connect (add initial delay)
- Mismatched QoS settings causing silent failures
- Not handling backpressure in high-frequency publishers
```

### 3. For Software Advanced
- Deep dive into internals
- Performance optimization techniques
- Advanced patterns and architectures
- Integration with other systems
- Research-level concepts

**Example:**
```markdown
<!-- Personalized for Advanced -->
**Advanced Publisher Patterns:**

\`\`\`python
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

class HighPerformancePublisher(Node):
    def __init__(self):
        super().__init__('hp_publisher')
        
        # Separate callback groups for parallel execution
        self.cb_group1 = MutuallyExclusiveCallbackGroup()
        self.cb_group2 = MutuallyExclusiveCallbackGroup()
        
        # Zero-copy publishing for large messages
        self.publisher = self.create_publisher(
            Image,
            'camera/image',
            10,
            callback_group=self.cb_group1
        )
        
        # Use intra-process communication for same-process subscribers
        self.get_logger().info(
            f'Intra-process: {self.publisher.intra_process_enabled}'
        )
\`\`\`

**Performance Optimization:**
1. **Zero-Copy Transport**: Use shared memory for large messages
2. **Callback Groups**: Parallelize callbacks with MultiThreadedExecutor
3. **Message Pooling**: Reuse message objects to reduce allocations
4. **DDS Tuning**: Configure FastDDS/CycloneDDS for your network

**Research Applications:**
- Real-time control loops with deterministic latency
- High-bandwidth sensor fusion (LiDAR + cameras)
- Distributed multi-robot systems
```

### 4. Hardware Background Personalization

#### For Hardware Beginners
- Explain physical concepts clearly
- Include hardware setup instructions
- Show wiring diagrams
- Discuss safety considerations
- Recommend beginner-friendly hardware

**Example:**
```markdown
**Hardware Setup: Connecting a Servo Motor**

\`\`\`
Servo Motor Wiring:
┌─────────────┐
│   Servo     │
│  ┌───┬───┬──┤
│  │ R │ B │ Y│  R = Red (Power, 5V)
│  └───┴───┴──┤  B = Brown/Black (Ground)
└─────────────┘  Y = Yellow/White (Signal)

Connect to Arduino:
- Red → 5V pin
- Brown → GND pin  
- Yellow → Digital Pin 9
\`\`\`

**Safety First:**
⚠️ Always disconnect power before wiring
⚠️ Check voltage requirements (most servos use 5V)
⚠️ Don't exceed current limits of your power supply
```

#### For Hardware Intermediates
- Focus on system integration
- Discuss sensor selection
- Cover calibration procedures
- Include troubleshooting guides

#### For Hardware Advanced
- Advanced sensor fusion
- Custom hardware integration
- PCB design considerations
- Real-time constraints

### 5. Programming Language Adaptations

#### For Python Developers
- Use Python idioms
- Leverage Python libraries
- Show Pythonic patterns

#### For C++ Developers
- Include C++ alternatives
- Discuss performance implications
- Show memory management

#### For JavaScript/Web Developers
- Draw parallels to web concepts
- Use familiar terminology
- Show web integration examples

## Personalization Workflow

### Step 1: Analyze User Profile
```python
user_profile = {
    'software_level': 'beginner',
    'hardware_level': 'intermediate',
    'languages': ['python', 'javascript'],
    'robotics_experience': 'none',
    'interests': ['autonomous navigation', 'computer vision']
}
```

### Step 2: Identify Content Sections
- Code examples
- Conceptual explanations
- Hardware discussions
- Practical exercises

### Step 3: Apply Transformations
- Adjust explanation depth
- Add/remove technical details
- Include relevant analogies
- Customize code examples
- Add appropriate warnings/tips

### Step 4: Add Personalized Callouts
```markdown
:::tip For Python Developers
This pattern is similar to decorators in Python. The lifecycle callbacks act like hooks that get called at specific times.
:::

:::info Your Background
Since you have JavaScript experience, think of ROS 2 nodes like Express.js servers - they handle requests and emit events.
:::

:::warning Hardware Note
As a hardware beginner, make sure to use a regulated power supply. The servo motors can draw significant current.
:::
```

### Step 5: Customize Examples
- Use user's preferred language
- Reference their interests
- Match complexity to level
- Include relevant applications

## Example Personalization

### Original Content:
```markdown
## Creating a ROS 2 Publisher

Use the `create_publisher()` method to create a publisher.

\`\`\`python
self.publisher = self.create_publisher(String, 'topic', 10)
\`\`\`
```

### Personalized for Beginner Software + Intermediate Hardware:
```markdown
## Creating a ROS 2 Publisher

**What You'll Learn:**  
How to broadcast sensor data from your robot to other parts of the system.

**Real-World Example:**  
Imagine your robot has a temperature sensor. You want to publish this data so other nodes can monitor it and trigger cooling if needed.

### Step-by-Step Guide

**1. Import Required Libraries**
\`\`\`python
from rclpy.node import Node
from std_msgs.msg import Float32  # For temperature data
\`\`\`

**2. Create the Publisher**
\`\`\`python
# Inside your __init__ method
self.temp_publisher = self.create_publisher(
    Float32,              # Message type (floating point number)
    'robot/temperature',  # Topic name (like a channel)
    10                    # Queue size (buffer for messages)
)
\`\`\`

**3. Publish Data**
\`\`\`python
def publish_temperature(self, temp_celsius):
    """Send temperature reading to subscribers."""
    msg = Float32()
    msg.data = temp_celsius
    self.temp_publisher.publish(msg)
    self.get_logger().info(f'Published temp: {temp_celsius}°C')
\`\`\`

**Hardware Connection:**
Since you're working with sensors, here's how to connect a typical temperature sensor (like TMP36):

\`\`\`
TMP36 → Arduino/Raspberry Pi
Pin 1 (Power) → 5V
Pin 2 (Signal) → Analog Pin A0
Pin 3 (Ground) → GND
\`\`\`

:::tip Sensor Reading
Read the analog value and convert to Celsius:
\`\`\`python
voltage = analog_value * (5.0 / 1024.0)
temp_celsius = (voltage - 0.5) * 100
\`\`\`
:::
```

## API Integration

### Backend Endpoint
```python
@router.post("/personalize-content")
async def personalize_content(
    chapter_id: str,
    user_id: str,
    db: Session = Depends(get_db)
):
    """Generate personalized chapter content."""
    
    # Get user profile
    user = db.query(User).filter(User.id == user_id).first()
    
    # Get original content
    chapter = get_chapter_content(chapter_id)
    
    # Generate personalized version
    personalized = await generate_personalized_content(
        content=chapter,
        software_level=user.software_level,
        hardware_level=user.hardware_level,
        languages=user.programming_languages,
        interests=user.interests
    )
    
    # Cache for future use
    cache_personalized_content(user_id, chapter_id, personalized)
    
    return {"content": personalized}
```

### Frontend Integration
```typescript
async function loadPersonalizedContent(chapterId: string) {
    const response = await fetch('/api/v1/personalize-content', {
        method: 'POST',
        headers: {
            'Authorization': `Bearer ${token}`,
            'Content-Type': 'application/json'
        },
        body: JSON.stringify({ chapter_id: chapterId })
    });
    
    const data = await response.json();
    return data.content;
}
```

## Success Criteria
- [ ] Content matches user's experience level
- [ ] Examples use familiar concepts
- [ ] Appropriate level of detail
- [ ] Relevant hardware references
- [ ] Engaging and accessible
- [ ] Maintains technical accuracy
- [ ] Preserves learning objectives
