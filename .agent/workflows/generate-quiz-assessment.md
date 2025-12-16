---
description: Generate comprehensive quizzes and assessments for textbook chapters
---

# Agent Skill: Generate Quiz & Assessment

This workflow creates comprehensive assessments to test student understanding of chapter content.

## Input Requirements
- **Chapter Number**: Which chapter this assessment is for
- **Chapter Title**: The chapter title
- **Key Concepts**: 5-10 main concepts covered
- **Difficulty Level**: Beginner, Intermediate, or Advanced
- **Learning Objectives**: The chapter's learning objectives

## Assessment Structure

### 1. Multiple Choice Questions (5-7 questions)
Each MCQ should:
- Test a specific concept
- Have 4 options (A, B, C, D)
- Include one correct answer
- Have plausible distractors
- Include explanation for correct answer

**Example:**
```markdown
**Question 1:** What is the primary purpose of a ROS 2 node?

A) To store robot configuration files  
B) To provide a modular unit of computation in a robot system  
C) To visualize robot data in 3D  
D) To compile C++ code for robots  

**Correct Answer:** B

**Explanation:** A ROS 2 node is a process that performs computation. Nodes are the fundamental building blocks of ROS 2 applications, allowing for modular, distributed robot systems. Each node should have a single, well-defined purpose.
```

### 2. True/False Questions (5 questions)
- Clear, unambiguous statements
- Test understanding of key facts
- Include explanations

**Example:**
```markdown
**Question 6:** True or False: In ROS 2, topics use a request-response communication pattern.

**Answer:** False

**Explanation:** Topics use a publish-subscribe pattern for one-way, asynchronous communication. Services use the request-response pattern for synchronous, two-way communication.
```

### 3. Short Answer Questions (3-5 questions)
- Require explanation in 2-4 sentences
- Test deeper understanding
- Include sample answers

**Example:**
```markdown
**Question 11:** Explain the difference between a ROS 2 topic and a ROS 2 service. When would you use each?

**Sample Answer:**  
Topics use a publish-subscribe pattern for continuous, one-way data streaming (e.g., sensor data), while services use a request-response pattern for occasional, two-way communication (e.g., triggering an action). Use topics when you need to broadcast data to multiple subscribers or when data flows continuously. Use services when you need a response to a specific request or when the interaction is transactional.
```

### 4. Code Analysis Questions (2-3 questions)
- Provide code snippet
- Ask students to identify issues or explain behavior
- Include detailed answers

**Example:**
```markdown
**Question 14:** What is wrong with this ROS 2 publisher code?

\`\`\`python
import rclpy
from std_msgs.msg import String

def main():
    rclpy.init()
    node = rclpy.create_node('publisher')
    pub = node.create_publisher(String, 'topic', 10)
    
    msg = String()
    msg.data = 'Hello'
    pub.publish(msg)
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()
\`\`\`

**Issues:**
1. The node is not spinning, so callbacks won't be processed
2. The message is published only once before shutdown
3. No error handling
4. Node is not properly destroyed before shutdown

**Corrected Version:**
\`\`\`python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class PublisherNode(Node):
    def __init__(self):
        super().__init__('publisher')
        self.pub = self.create_publisher(String, 'topic', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
    
    def timer_callback(self):
        msg = String()
        msg.data = 'Hello'
        self.pub.publish(msg)

def main():
    rclpy.init()
    node = PublisherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
\`\`\`
```

### 5. Practical Coding Challenge (1 challenge)
- Real-world problem
- Clear requirements
- Starter code template
- Complete solution
- Test cases

**Example:**
```markdown
**Coding Challenge:** Create a ROS 2 node that subscribes to laser scan data and publishes obstacle detection alerts.

**Requirements:**
1. Subscribe to `/scan` topic (sensor_msgs/LaserScan)
2. Detect obstacles within 1 meter
3. Publish alert to `/obstacle_alert` (std_msgs/Bool)
4. Log obstacle distance to console

**Starter Template:**
\`\`\`python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool

class ObstacleDetector(Node):
    def __init__(self):
        super().__init__('obstacle_detector')
        # TODO: Create subscriber
        # TODO: Create publisher
        # TODO: Set detection threshold
    
    def scan_callback(self, msg):
        # TODO: Process laser scan data
        # TODO: Detect obstacles
        # TODO: Publish alert
        pass

def main():
    # TODO: Initialize and spin node
    pass

if __name__ == '__main__':
    main()
\`\`\`

**Solution:**
\`\`\`python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool

class ObstacleDetector(Node):
    def __init__(self):
        super().__init__('obstacle_detector')
        
        # Parameters
        self.declare_parameter('threshold', 1.0)
        self.threshold = self.get_parameter('threshold').value
        
        # Subscriber
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )
        
        # Publisher
        self.alert_pub = self.create_publisher(
            Bool,
            '/obstacle_alert',
            10
        )
        
        self.get_logger().info(
            f'Obstacle detector started (threshold: {self.threshold}m)'
        )
    
    def scan_callback(self, msg):
        # Find minimum distance in scan
        valid_ranges = [r for r in msg.ranges if r > 0]
        
        if not valid_ranges:
            return
        
        min_distance = min(valid_ranges)
        
        # Check if obstacle detected
        obstacle_detected = min_distance < self.threshold
        
        # Publish alert
        alert_msg = Bool()
        alert_msg.data = obstacle_detected
        self.alert_pub.publish(alert_msg)
        
        # Log if obstacle detected
        if obstacle_detected:
            self.get_logger().warn(
                f'Obstacle detected at {min_distance:.2f}m!'
            )

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleDetector()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
\`\`\`

**Test Cases:**
1. No obstacles: All ranges > 1.0m → alert = False
2. Close obstacle: Min range = 0.5m → alert = True
3. Edge case: Min range = 1.0m → alert = False (threshold is <, not <=)
```

### 6. Project Extension (Optional Advanced Task)
- Builds on coding challenge
- Adds complexity
- Encourages creativity
- Provides guidance, not full solution

**Example:**
```markdown
**Project Extension:** Enhance the obstacle detector with the following features:

1. **Multi-zone Detection:** Divide the scan into left, center, and right zones. Publish separate alerts for each zone.

2. **Distance Tracking:** Track the closest obstacle over time and publish its velocity (approaching/receding).

3. **Visualization:** Create a simple visualization showing obstacle positions using RViz markers.

4. **Parameter Server:** Make the threshold configurable via ROS 2 parameters and allow runtime updates.

**Hints:**
- Use `geometry_msgs/Point` for zone-based alerts
- Calculate velocity using time difference between scans
- Use `visualization_msgs/Marker` for RViz
- Implement parameter callback for dynamic reconfiguration
```

## Question Distribution by Bloom's Taxonomy

- **Remember** (20%): Recall facts, terms, concepts
- **Understand** (25%): Explain ideas, summarize
- **Apply** (25%): Use knowledge in new situations
- **Analyze** (15%): Break down information, find patterns
- **Evaluate** (10%): Justify decisions, critique
- **Create** (5%): Design, build, produce

## Execution Steps

1. **Review Chapter Content**
   - Identify key concepts
   - Note important code examples
   - List common misconceptions

2. **Create Question Bank**
   - Write 7-10 MCQs covering main topics
   - Design 5 T/F questions for key facts
   - Develop 3-5 short answer questions
   - Create 2-3 code analysis problems

3. **Design Coding Challenge**
   - Choose practical problem
   - Write clear requirements
   - Create starter template
   - Implement complete solution
   - Design test cases

4. **Develop Project Extension**
   - Build on coding challenge
   - Add 3-4 enhancement ideas
   - Provide implementation hints
   - Suggest learning resources

5. **Review & Validate**
   - Check question clarity
   - Verify correct answers
   - Test code solutions
   - Ensure appropriate difficulty

6. **Create Answer Key**
   - Separate file with all answers
   - Include detailed explanations
   - Provide grading rubrics
   - Add common mistakes to watch for

## File Structure
```
frontend/docs/module-X/
├── chapter-Y.md                    # Main chapter content
├── assessments/
│   ├── chapter-Y-quiz.md          # Quiz questions
│   ├── chapter-Y-answers.md       # Answer key (hidden)
│   └── chapter-Y-challenge.md     # Coding challenge
```

## Success Criteria
- [ ] 15-20 total questions
- [ ] Covers all learning objectives
- [ ] Mix of difficulty levels
- [ ] All code solutions tested
- [ ] Clear explanations provided
- [ ] Aligned with chapter content
- [ ] Includes practical application
- [ ] Answer key complete
