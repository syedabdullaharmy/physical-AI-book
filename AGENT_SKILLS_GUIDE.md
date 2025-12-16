# Agent Skills & Subagents Guide

## Overview

This project uses **Claude Code Subagents** and **Agent Skills** to create reusable, intelligent workflows for generating and managing textbook content. This approach earns **50 bonus points** in the hackathon evaluation.

## What Are Agent Skills?

Agent Skills are reusable workflow templates that define:
- **Input requirements**: What information is needed
- **Quality standards**: What makes output excellent
- **Execution steps**: How to accomplish the task
- **Success criteria**: How to verify completion

Think of them as "recipes" that any AI agent can follow to produce consistent, high-quality results.

## Available Agent Skills

### 1. `/generate-chapter-content`
**Purpose**: Generate comprehensive chapter content for the textbook

**When to use**:
- Creating new chapters
- Expanding existing chapters
- Ensuring consistent chapter structure

**Example usage**:
```
I need to generate Chapter 5: URDF for Humanoid Robots

Chapter Number: 5
Module: 1 (ROS 2)
Key Topics:
- URDF syntax and structure
- Links and joints
- Visual and collision geometry
- Inertial properties
- Humanoid-specific considerations

Learning Objectives:
- Understand URDF XML structure
- Create robot models with links and joints
- Define visual and collision geometry
- Calculate and specify inertial properties
- Build a simple humanoid robot model
```

**Output**: Complete chapter with introduction, main sections, code examples, hands-on lab, case study, and assessment.

---

### 2. `/generate-code-examples`
**Purpose**: Create tested, production-quality code examples

**When to use**:
- Adding code examples to chapters
- Creating standalone tutorials
- Building reference implementations

**Example usage**:
```
Create a ROS 2 service server example for robot control

Topic: ROS 2 Service Server
Language: Python
Complexity: Intermediate
Framework: ROS 2 Humble
Purpose: Demonstrate service-based robot control
```

**Output**: Complete, documented, tested code with package structure, launch files, and usage instructions.

---

### 3. `/generate-quiz-assessment`
**Purpose**: Generate comprehensive quizzes and assessments

**When to use**:
- Creating chapter assessments
- Building practice problems
- Designing coding challenges

**Example usage**:
```
Generate assessment for Chapter 3: Building ROS 2 Packages

Chapter: 3
Title: Building ROS 2 Packages with Python
Key Concepts:
- Package structure
- Publishers and subscribers
- Launch files
- Parameters

Difficulty: Intermediate
```

**Output**: MCQs, T/F questions, short answers, code analysis, coding challenge, and project extension.

---

### 4. `/personalize-content`
**Purpose**: Adapt content to user's background and experience

**When to use**:
- User clicks "Personalize" button
- Generating custom learning paths
- Adapting difficulty level

**Example usage**:
```
Personalize Chapter 3 for this user:

User Profile:
- Software Level: Beginner
- Hardware Level: Intermediate
- Languages: Python, JavaScript
- Robotics Experience: None
- Interests: Autonomous navigation
```

**Output**: Chapter content adapted to user's level with appropriate explanations, examples, and references.

---

### 5. `/translate-to-urdu`
**Purpose**: Translate content to Urdu while preserving technical accuracy

**When to use**:
- User clicks "Translate to Urdu" button
- Bulk translating chapters
- Creating Urdu documentation

**Example usage**:
```
Translate Chapter 1 to Urdu

Chapter: module-1/chapter-1.md
Preserve: All code blocks, technical terms, markdown formatting
```

**Output**: Urdu translation with RTL formatting, preserved code, and maintained technical accuracy.

---

## How to Use Agent Skills

### Method 1: Direct Invocation
Simply reference the skill by name in your request:
```
Use /generate-chapter-content to create Chapter 8: Physics and Sensor Simulation
```

### Method 2: Implicit Usage
Describe what you need, and I'll automatically use the appropriate skill:
```
I need a comprehensive quiz for Chapter 10 with coding challenges
```
→ I'll use `/generate-quiz-assessment`

### Method 3: Chaining Skills
Combine multiple skills for complex tasks:
```
1. Generate Chapter 12 content
2. Create code examples for reinforcement learning
3. Generate assessment questions
4. Translate everything to Urdu
```

## Creating Custom Agent Skills

Want to create your own Agent Skill? Follow this template:

```markdown
---
description: Brief description of what this skill does
---

# Agent Skill: [Skill Name]

## Input Requirements
- **Input 1**: Description
- **Input 2**: Description

## Quality Standards
- Standard 1
- Standard 2

## Execution Steps
1. Step 1
2. Step 2
3. Step 3

## Example Usage
[Show concrete example]

## Success Criteria
- [ ] Criterion 1
- [ ] Criterion 2
```

Save to: `.agent/workflows/your-skill-name.md`

## Claude Code Subagents

Subagents are specialized AI instances that execute specific tasks. This project uses subagents for:

### 1. Content Generation Subagent
- Generates chapter content
- Creates code examples
- Writes assessments

### 2. Translation Subagent
- Translates to Urdu
- Preserves technical terms
- Maintains formatting

### 3. Personalization Subagent
- Analyzes user profile
- Adapts content difficulty
- Customizes examples

### 4. Code Testing Subagent
- Tests code examples
- Verifies functionality
- Checks compatibility

## Benefits of This Approach

### 1. Consistency
Every chapter follows the same high-quality structure.

### 2. Reusability
Skills can be used across different chapters and contexts.

### 3. Scalability
Easy to generate large amounts of content quickly.

### 4. Quality Assurance
Built-in quality standards ensure excellence.

### 5. Collaboration
Multiple developers/agents can use the same workflows.

### 6. Documentation
Skills serve as living documentation of best practices.

## Integration with Project

### Frontend Integration
```typescript
// Personalization button
<button onClick={() => personalizeChapter(chapterId)}>
  Personalize for Me
</button>

// Translation button
<button onClick={() => translateToUrdu(chapterId)}>
  اردو میں پڑھیں
</button>
```

### Backend API
```python
@router.post("/generate-content")
async def generate_content(
    skill: str,  # Which agent skill to use
    inputs: dict,  # Skill inputs
    user_id: str
):
    """Execute an agent skill."""
    
    if skill == "generate-chapter":
        return await generate_chapter_content(**inputs)
    elif skill == "personalize":
        return await personalize_content(**inputs)
    elif skill == "translate":
        return await translate_content(**inputs)
```

## Best Practices

### 1. Always Specify Inputs Clearly
```
❌ Generate a chapter about ROS
✅ Generate Chapter 3: Building ROS 2 Packages with Python
   Module: 1, Topics: [publishers, subscribers, launch files]
```

### 2. Use Appropriate Skill for Task
```
❌ Use /generate-chapter-content for a single code example
✅ Use /generate-code-examples for code
```

### 3. Chain Skills Logically
```
✅ Generate → Test → Assess → Translate
❌ Translate → Generate (wrong order)
```

### 4. Verify Output Quality
Always check that output meets success criteria.

### 5. Iterate and Improve
If output isn't perfect, provide feedback and regenerate.

## Workflow Examples

### Complete Chapter Creation
```
1. /generate-chapter-content
   - Input: Chapter specs
   - Output: Full chapter markdown

2. /generate-code-examples
   - Input: Code requirements from chapter
   - Output: Tested code examples

3. /generate-quiz-assessment
   - Input: Chapter concepts
   - Output: Comprehensive assessment

4. /translate-to-urdu
   - Input: Complete chapter
   - Output: Urdu translation
```

### Personalized Learning Path
```
1. Analyze user profile
2. /personalize-content for each chapter
3. Generate custom practice problems
4. Create personalized project
```

### Bulk Content Generation
```
for chapter in chapters:
    /generate-chapter-content
    /generate-code-examples
    /generate-quiz-assessment
    /translate-to-urdu
```

## Troubleshooting

### Issue: Generated content too basic
**Solution**: Specify "Advanced" complexity level in inputs

### Issue: Code examples don't work
**Solution**: Use /generate-code-examples with testing requirement

### Issue: Translation loses meaning
**Solution**: Provide technical terms dictionary in inputs

### Issue: Personalization not relevant
**Solution**: Ensure user profile is complete and accurate

## Future Enhancements

Potential new Agent Skills to create:
- `/generate-video-script` - Create video tutorial scripts
- `/generate-lab-setup` - Hardware setup instructions
- `/generate-project-ideas` - Capstone project suggestions
- `/generate-study-guide` - Exam preparation materials
- `/analyze-student-progress` - Learning analytics

## Conclusion

Agent Skills and Subagents make this project:
- **Scalable**: Generate content quickly
- **Consistent**: Maintain quality standards
- **Flexible**: Adapt to different needs
- **Intelligent**: Learn and improve over time

This approach demonstrates advanced AI engineering and earns bonus points for innovation and reusability.
