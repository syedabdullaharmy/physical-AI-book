---
description: Generate comprehensive chapter content for Physical AI & Humanoid Robotics textbook
---

# Agent Skill: Generate Chapter Content

This workflow generates high-quality, comprehensive chapter content for the Physical AI & Humanoid Robotics textbook.

## Input Requirements
- **Chapter Number**: The chapter number (1-17)
- **Chapter Title**: The title of the chapter
- **Module**: Which module this belongs to (1-4)
- **Key Topics**: List of 3-5 key topics to cover
- **Learning Objectives**: 3-5 specific learning objectives

## Output Structure
Each chapter should follow this exact structure:

### 1. Chapter Header
```markdown
# Chapter X: [Title]

**Module Y: [Module Name]**

## Learning Objectives
By the end of this chapter, you will be able to:
- [Objective 1]
- [Objective 2]
- [Objective 3]
```

### 2. Introduction (300-500 words)
- Context and motivation
- Real-world applications
- Connection to previous chapters
- Chapter roadmap

### 3. Main Content Sections (4-6 sections)
Each section should include:
- **Clear explanations** (500-800 words per section)
- **Code examples** with detailed comments
- **Diagrams/visualizations** (describe what should be shown)
- **Practical examples** from robotics
- **Best practices** and common pitfalls

### 4. Hands-On Lab Exercise
- **Objective**: What students will build
- **Prerequisites**: Required setup
- **Step-by-step instructions** (10-15 steps)
- **Complete code** with explanations
- **Expected output** and verification steps
- **Troubleshooting** common issues

### 5. Real-World Application
- **Case study** from industry or research
- **Implementation details**
- **Results and impact**
- **Lessons learned**

### 6. Summary
- Key takeaways (bullet points)
- Connections to next chapter
- Additional resources

### 7. Assessment
- **Multiple Choice Questions** (5 questions)
- **Short Answer Questions** (3 questions)
- **Coding Challenge** (1 practical problem)
- **Project Extension** (optional advanced task)

## Quality Standards

### Technical Accuracy
- All code must be tested and functional
- Use latest versions of libraries (ROS 2 Humble, Python 3.10+)
- Follow industry best practices
- Include proper error handling

### Code Examples
- Use Python for ROS 2 examples
- Include complete, runnable code
- Add detailed comments explaining each section
- Show both basic and advanced usage
- Include package.xml and setup.py when relevant

### Writing Style
- Clear, concise, and engaging
- Use active voice
- Define technical terms on first use
- Include analogies for complex concepts
- Balance theory with practice

### Formatting
- Use proper markdown syntax
- Include code blocks with language tags
- Use callouts for important notes:
  - :::tip for helpful hints
  - :::warning for common mistakes
  - :::danger for critical issues
  - :::info for additional context

## Execution Steps

1. **Research Phase**
   - Review official documentation for the topic
   - Check latest best practices
   - Identify common student challenges
   - Find relevant real-world examples

2. **Content Generation**
   - Write introduction with clear motivation
   - Develop each main section with examples
   - Create hands-on lab with tested code
   - Find or create case study
   - Write comprehensive summary

3. **Code Development**
   - Write all code examples
   - Test each example locally
   - Add detailed comments
   - Include error handling
   - Verify compatibility

4. **Assessment Creation**
   - Design questions testing key concepts
   - Create coding challenge
   - Develop project extension
   - Include answer key (separate file)

5. **Review & Polish**
   - Check for technical accuracy
   - Verify all code runs
   - Ensure consistent formatting
   - Add cross-references
   - Proofread for clarity

## Example Usage

To generate Chapter 3: Building ROS 2 Packages with Python:

```
Chapter Number: 3
Chapter Title: Building ROS 2 Packages with Python
Module: 1 (The Robotic Nervous System)
Key Topics:
- Package structure and organization
- Python nodes and lifecycle
- Publishers and subscribers
- Services and actions
- Launch files

Learning Objectives:
- Create and structure ROS 2 Python packages
- Implement publisher and subscriber nodes
- Design service and action servers/clients
- Write launch files for multi-node systems
- Follow ROS 2 Python best practices
```

## File Naming Convention
- Main content: `frontend/docs/module-X/chapter-Y.md`
- Code examples: `frontend/docs/module-X/code/chapter-Y/`
- Assets: `frontend/docs/module-X/assets/chapter-Y/`

## Success Criteria
- [ ] Chapter is 3000-5000 words
- [ ] All code examples are tested and working
- [ ] Includes at least 3 code examples
- [ ] Has complete hands-on lab
- [ ] Contains real-world case study
- [ ] Assessment has 5+ questions
- [ ] Follows markdown formatting standards
- [ ] Links to relevant resources
- [ ] Includes visual descriptions for diagrams
