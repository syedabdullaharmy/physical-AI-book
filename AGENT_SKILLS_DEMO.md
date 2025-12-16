# Agent Skills Demo: Quick Start Examples

## Example 1: Generate Chapter Content

### Request:
```
Use /generate-chapter-content to create Chapter 11: AI-Powered Perception

Chapter Number: 11
Chapter Title: AI-Powered Perception
Module: 3 (NVIDIA Isaac)
Key Topics:
- Computer vision fundamentals
- Object detection with YOLO
- Semantic segmentation
- Depth estimation
- Isaac Sim perception sensors

Learning Objectives:
- Understand computer vision pipeline in robotics
- Implement object detection for robot perception
- Use semantic segmentation for scene understanding
- Process depth data for 3D perception
- Integrate perception with Isaac Sim
```

### Expected Output:
- Complete chapter markdown (3000-5000 words)
- Introduction with real-world context
- 5-6 main sections with detailed explanations
- Code examples for each perception technique
- Hands-on lab: Building a perception pipeline
- Real-world case study
- Comprehensive assessment

---

## Example 2: Generate Code Examples

### Request:
```
Use /generate-code-examples to create a ROS 2 object detection node

Topic: Object Detection with YOLO in ROS 2
Language: Python
Complexity: Intermediate
Framework: ROS 2 Humble + OpenCV + YOLO
Purpose: Real-time object detection from camera feed
```

### Expected Output:
```python
"""
ROS 2 Object Detection Node using YOLO.

This node subscribes to camera images, performs object detection,
and publishes detected objects with bounding boxes.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray, Detection2D
from cv_bridge import CvBridge
import cv2
import numpy as np

class YOLODetector(Node):
    """Object detection node using YOLO."""
    
    def __init__(self):
        super().__init__('yolo_detector')
        
        # Parameters
        self.declare_parameter('model_path', 'yolov5s.pt')
        self.declare_parameter('confidence_threshold', 0.5)
        
        # ... (complete implementation)
```

---

## Example 3: Generate Quiz

### Request:
```
Use /generate-quiz-assessment for Chapter 3: Building ROS 2 Packages

Chapter: 3
Title: Building ROS 2 Packages with Python
Key Concepts:
- Package structure
- Publishers and subscribers
- Launch files
- Parameters
- Best practices

Difficulty: Intermediate
```

### Expected Output:
- 5-7 Multiple Choice Questions
- 5 True/False Questions
- 3-5 Short Answer Questions
- 2-3 Code Analysis Problems
- 1 Practical Coding Challenge
- 1 Project Extension
- Complete answer key

---

## Example 4: Personalize Content

### Request:
```
Use /personalize-content for Chapter 5

Chapter: module-1/chapter-5.md (URDF for Humanoid Robots)

User Profile:
- Software Level: Beginner
- Hardware Level: Advanced
- Languages: Python, C++
- Robotics Experience: Built hobby robots
- Interests: Humanoid locomotion, bipedal walking
```

### Expected Output:
- Simplified software explanations
- Advanced hardware integration details
- Examples in Python (user's preference)
- Focus on humanoid locomotion applications
- Hardware-specific tips and warnings
- Links to bipedal walking resources

---

## Example 5: Translate to Urdu

### Request:
```
Use /translate-to-urdu for Chapter 1

Chapter: module-1/chapter-1.md
Preserve: All code blocks, technical terms, markdown formatting
```

### Expected Output:
```markdown
# باب 1: ROS 2 Architecture

## تعارف

ROS 2 (Robot Operating System 2) ایک جدید robotics middleware ہے...

### Publisher بنانا

\`\`\`python
# یہ code ایک publisher بناتا ہے
self.publisher = self.create_publisher(String, 'topic', 10)
\`\`\`

:::tip بہترین طریقہ
ہمیشہ descriptive topic names استعمال کریں۔
:::
```

---

## Chaining Multiple Skills

### Request:
```
Create complete content for Chapter 12:

1. Generate chapter content
2. Create 3 code examples (beginner, intermediate, advanced)
3. Generate comprehensive assessment
4. Translate to Urdu
```

### Workflow:
```
Step 1: /generate-chapter-content
→ Chapter 12: Reinforcement Learning for Robots

Step 2: /generate-code-examples (3x)
→ Example 1: Basic Q-Learning
→ Example 2: DQN for Navigation
→ Example 3: PPO for Manipulation

Step 3: /generate-quiz-assessment
→ 20 questions + coding challenge

Step 4: /translate-to-urdu
→ Complete Urdu version
```

---

## Bulk Operations

### Generate All Module 4 Chapters:
```
For chapters 14-17:
  /generate-chapter-content
  /generate-code-examples
  /generate-quiz-assessment
  /translate-to-urdu
```

### Personalize Entire Textbook:
```
For user_profile in users:
  For chapter in chapters:
    /personalize-content
```

---

## Testing a Skill

### Test Chapter Generation:
```
Use /generate-chapter-content for a test chapter

Chapter Number: TEST
Chapter Title: Test Chapter - ROS 2 Basics
Module: 1
Key Topics: [nodes, topics]
Learning Objectives: [understand nodes, create publishers]
```

Verify output has:
- [ ] Proper markdown formatting
- [ ] Code examples with syntax highlighting
- [ ] Hands-on lab section
- [ ] Assessment questions
- [ ] 3000+ words

---

## Common Use Cases

### 1. New Chapter Creation
```
/generate-chapter-content → /generate-code-examples → /generate-quiz-assessment
```

### 2. Content Enhancement
```
/generate-code-examples (add more examples to existing chapter)
```

### 3. User-Specific Content
```
/personalize-content (adapt for specific user)
```

### 4. Localization
```
/translate-to-urdu (create Urdu version)
```

### 5. Assessment Creation
```
/generate-quiz-assessment (create practice tests)
```

---

## Tips for Best Results

### ✅ Do:
- Provide complete, specific inputs
- Specify difficulty level
- Include learning objectives
- Request specific frameworks/versions
- Ask for tested code

### ❌ Don't:
- Give vague requirements
- Mix multiple unrelated topics
- Skip important context
- Forget to specify target audience

---

## Troubleshooting

### Issue: Output too basic
**Solution**: Specify "Advanced" complexity

### Issue: Code doesn't work
**Solution**: Request "tested, production-ready code"

### Issue: Missing sections
**Solution**: Explicitly list required sections

### Issue: Wrong framework version
**Solution**: Specify exact version (e.g., "ROS 2 Humble")

---

## Next Steps

1. **Try a skill**: Pick one and test it
2. **Review output**: Check against success criteria
3. **Iterate**: Provide feedback and regenerate
4. **Integrate**: Use in your workflow
5. **Create custom**: Build your own skills

---

## Quick Reference

| Skill | Use When | Output |
|-------|----------|--------|
| `/generate-chapter-content` | Creating chapters | Full chapter markdown |
| `/generate-code-examples` | Need code samples | Tested code + docs |
| `/generate-quiz-assessment` | Creating tests | Questions + answers |
| `/personalize-content` | User-specific | Adapted content |
| `/translate-to-urdu` | Localization | Urdu translation |

---

Ready to use Agent Skills? Just reference them in your requests!
