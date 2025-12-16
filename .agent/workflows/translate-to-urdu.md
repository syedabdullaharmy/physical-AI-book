---
description: Translate chapter content to Urdu while preserving technical accuracy
---

# Agent Skill: Translate to Urdu

This workflow translates English chapter content to Urdu while maintaining technical accuracy, code integrity, and proper RTL formatting.

## Input Requirements
- **Chapter Content**: Original English markdown
- **Chapter ID**: For file organization
- **Technical Terms**: List of terms to preserve in English

## Translation Guidelines

### 1. Preserve Technical Elements
**DO NOT translate:**
- Code blocks and inline code
- Command-line instructions
- File paths and URLs
- Technical acronyms (ROS, API, SDK, etc.)
- Library/package names
- Variable names
- Function names

**Example:**
```markdown
<!-- English -->
Use the `create_publisher()` method to create a ROS 2 publisher.

<!-- Urdu -->
ROS 2 publisher بنانے کے لیے `create_publisher()` method استعمال کریں۔
```

### 2. RTL (Right-to-Left) Formatting
- Main text flows right to left
- Code blocks remain left to right
- Numbers use Western numerals
- Preserve markdown structure

### 3. Technical Term Handling

#### Keep in English:
- ROS 2, Gazebo, Isaac Sim
- Publisher, Subscriber, Node
- URDF, SDF, YAML
- Python, C++, JavaScript

#### Translate with English in parentheses:
- Robot → روبوٹ (Robot)
- Sensor → سینسر (Sensor)
- Simulation → سمیولیشن (Simulation)
- Algorithm → الگورتھم (Algorithm)

### 4. Code Comments Translation
Translate comments while keeping code unchanged:

```python
# English
def calculate_distance(x, y):
    """Calculate Euclidean distance between two points."""
    return math.sqrt(x**2 + y**2)

# Urdu
def calculate_distance(x, y):
    """دو نقاط کے درمیان یوکلیڈین فاصلہ شمار کریں۔"""
    return math.sqrt(x**2 + y**2)
```

## Translation Workflow

### Step 1: Prepare Content
1. Extract code blocks (preserve as-is)
2. Identify technical terms
3. Mark sections for translation
4. Note special formatting

### Step 2: Translate Text Sections
- Translate headings
- Translate paragraphs
- Translate list items
- Translate table content
- Keep technical terms in English

### Step 3: Translate Callouts
```markdown
<!-- English -->
:::tip Best Practice
Always initialize your node before creating publishers.
:::

<!-- Urdu -->
:::tip بہترین طریقہ
Publishers بنانے سے پہلے ہمیشہ اپنے node کو initialize کریں۔
:::
```

### Step 4: Handle Mixed Content
```markdown
<!-- English -->
The `rclpy.spin()` function keeps the node running until interrupted.

<!-- Urdu -->
`rclpy.spin()` function node کو چلاتا رہتا ہے جب تک کہ اسے روکا نہ جائے۔
```

### Step 5: Verify Formatting
- Check RTL rendering
- Verify code blocks intact
- Test links functionality
- Validate markdown syntax

## Example Translation

### Original (English):
```markdown
# Chapter 3: Building ROS 2 Packages with Python

## Learning Objectives
By the end of this chapter, you will be able to:
- Create and structure ROS 2 Python packages
- Implement publisher and subscriber nodes
- Write launch files for multi-node systems

## Introduction

ROS 2 packages are the fundamental building blocks of robot applications. A package contains nodes, libraries, configuration files, and other resources organized in a standardized structure.

### Package Structure

A typical ROS 2 Python package has the following structure:

\`\`\`
my_robot_package/
├── package.xml
├── setup.py
├── my_robot_package/
│   ├── __init__.py
│   └── my_node.py
└── launch/
    └── my_launch.py
\`\`\`

:::tip Best Practice
Always use descriptive names for your packages and nodes.
:::

## Creating a Publisher

Here's how to create a simple publisher:

\`\`\`python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher = self.create_publisher(String, 'topic', 10)
        
    def publish_message(self):
        msg = String()
        msg.data = 'Hello ROS 2'
        self.publisher.publish(msg)
\`\`\`
```

### Translated (Urdu):
```markdown
# باب 3: Python کے ساتھ ROS 2 Packages بنانا

## سیکھنے کے مقاصد
اس باب کے اختتام تک، آپ یہ کر سکیں گے:
- ROS 2 Python packages بنانا اور ان کی ساخت تیار کرنا
- Publisher اور subscriber nodes کو implement کرنا
- Multi-node systems کے لیے launch files لکھنا

## تعارف

ROS 2 packages روبوٹ ایپلیکیشنز کے بنیادی building blocks ہیں۔ ایک package میں nodes، libraries، configuration files، اور دیگر resources ہوتے ہیں جو ایک معیاری ڈھانچے میں منظم ہوتے ہیں۔

### Package کی ساخت

ایک عام ROS 2 Python package کی ساخت یہ ہوتی ہے:

\`\`\`
my_robot_package/
├── package.xml
├── setup.py
├── my_robot_package/
│   ├── __init__.py
│   └── my_node.py
└── launch/
    └── my_launch.py
\`\`\`

:::tip بہترین طریقہ
اپنے packages اور nodes کے لیے ہمیشہ واضح نام استعمال کریں۔
:::

## Publisher بنانا

یہاں ایک سادہ publisher بنانے کا طریقہ ہے:

\`\`\`python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher = self.create_publisher(String, 'topic', 10)
        
    def publish_message(self):
        msg = String()
        msg.data = 'Hello ROS 2'
        self.publisher.publish(msg)
\`\`\`
```

## Common Technical Terms Dictionary

| English | Urdu | Usage |
|---------|------|-------|
| Robot | روبوٹ | Keep both |
| Node | Node | Keep English |
| Publisher | Publisher | Keep English |
| Subscriber | Subscriber | Keep English |
| Topic | Topic | Keep English |
| Service | Service | Keep English |
| Package | Package | Keep English |
| Sensor | سینسر | Keep both |
| Motor | موٹر | Keep both |
| Simulation | سمیولیشن | Keep both |
| Algorithm | الگورتھم | Keep both |
| Function | فنکشن | Keep both |
| Variable | متغیر | Keep both |
| Loop | لوپ | Keep both |
| Condition | شرط | Translate |
| Error | خرابی | Translate |
| Warning | انتباہ | Translate |

## API Integration

### Backend Endpoint
```python
from mistralai.client import MistralClient
from mistralai.models.chat_completion import ChatMessage

@router.post("/translate-to-urdu")
async def translate_to_urdu(
    chapter_id: str,
    user_id: str,
    db: Session = Depends(get_db)
):
    """Translate chapter content to Urdu."""
    
    # Get original content
    chapter = get_chapter_content(chapter_id)
    
    # Check cache first
    cached = get_cached_translation(chapter_id, 'ur')
    if cached:
        return {"content": cached}
    
    # Translate using Mistral AI
    client = MistralClient(api_key=settings.MISTRAL_API_KEY)
    
    prompt = f"""
    Translate the following technical content to Urdu while:
    1. Preserving all code blocks exactly as-is
    2. Keeping technical terms in English
    3. Maintaining markdown formatting
    4. Using proper RTL formatting
    5. Translating comments in code
    
    Content:
    {chapter}
    """
    
    response = client.chat(
        model="mistral-large-latest",
        messages=[ChatMessage(role="user", content=prompt)]
    )
    
    translated = response.choices[0].message.content
    
    # Cache translation
    cache_translation(chapter_id, 'ur', translated)
    
    return {"content": translated}
```

### Frontend Integration
```typescript
async function loadUrduTranslation(chapterId: string) {
    const response = await fetch('/api/v1/translate-to-urdu', {
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

## File Organization
```
frontend/
├── docs/                          # English content
│   └── module-1/
│       └── chapter-1.md
└── i18n/
    └── ur/                        # Urdu translations
        └── docusaurus-plugin-content-docs/
            └── current/
                └── module-1/
                    └── chapter-1.md
```

## Quality Checklist
- [ ] All code blocks preserved exactly
- [ ] Technical terms kept in English
- [ ] RTL formatting correct
- [ ] Markdown syntax valid
- [ ] Links functional
- [ ] Images display correctly
- [ ] Callouts translated
- [ ] Tables formatted properly
- [ ] Code comments translated
- [ ] Meaning preserved accurately

## Success Criteria
- [ ] Content readable in Urdu
- [ ] Technical accuracy maintained
- [ ] Code remains executable
- [ ] RTL rendering correct
- [ ] No broken formatting
- [ ] Consistent terminology
- [ ] Natural language flow
