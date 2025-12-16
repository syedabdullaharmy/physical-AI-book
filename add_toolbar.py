"""
Script to add ChapterToolbar to all chapter markdown files.
"""

import os
import re

# Chapter titles mapping
CHAPTERS = {
    "module-1/chapter-1": "ROS 2 Architecture and Core Concepts",
    "module-1/chapter-2": "Nodes, Topics, and Services",
    "module-1/chapter-3": "Building ROS 2 Packages with Python",
    "module-1/chapter-4": "Launch Files and Parameter Management",
    "module-1/chapter-5": "URDF for Humanoid Robots",
    "module-2/chapter-6": "Gazebo Simulation Environment",
    "module-2/chapter-7": "URDF and SDF Formats",
    "module-2/chapter-8": "Physics and Sensor Simulation",
    "module-2/chapter-9": "Unity for Robot Visualization",
    "module-3/chapter-10": "NVIDIA Isaac SDK and Sim",
    "module-3/chapter-11": "AI-Powered Perception",
    "module-3/chapter-12": "Reinforcement Learning for Robots",
    "module-3/chapter-13": "Sim-to-Real Transfer",
    "module-4/chapter-14": "Humanoid Kinematics and Dynamics",
    "module-4/chapter-15": "Bipedal Locomotion",
    "module-4/chapter-16": "Manipulation and Grasping",
    "module-4/chapter-17": "Conversational Robotics with GPT",
}

def add_toolbar_to_chapter(file_path, chapter_id, chapter_title):
    """Add ChapterToolbar component to a chapter file."""
    
    with open(file_path, 'r', encoding='utf-8') as f:
        content = f.read()
    
    # Check if toolbar already added
    if 'ChapterToolbar' in content:
        print(f"✓ Toolbar already present in {chapter_id}")
        return False
    
    # Find the frontmatter and first heading
    pattern = r'(---\n.*?---\n\n)(# .*)'
    match = re.search(pattern, content, re.DOTALL)
    
    if not match:
        print(f"✗ Could not find frontmatter in {chapter_id}")
        return False
    
    frontmatter = match.group(1)
    heading = match.group(2)
    rest = content[match.end():]
    
    # Create toolbar component
    toolbar = f"""import ChapterToolbar from '@site/src/components/ChapterToolbar';

<ChapterToolbar 
    chapterId="{chapter_id}" 
    chapterTitle="{chapter_title}" 
/>

"""
    
    # Reconstruct file
    new_content = frontmatter + toolbar + heading + rest
    
    # Write back
    with open(file_path, 'w', encoding='utf-8') as f:
        f.write(new_content)
    
    print(f"✓ Added toolbar to {chapter_id}")
    return True

def main():
    base_path = r"c:\Users\FA.COM\Pictures\Camera Roll\book\frontend\docs"
    
    updated = 0
    skipped = 0
    
    for chapter_id, chapter_title in CHAPTERS.items():
        file_path = os.path.join(base_path, f"{chapter_id}.md")
        
        if not os.path.exists(file_path):
            print(f"✗ File not found: {file_path}")
            continue
        
        if add_toolbar_to_chapter(file_path, chapter_id, chapter_title):
            updated += 1
        else:
            skipped += 1
    
    print(f"\n{'='*50}")
    print(f"Integration complete!")
    print(f"Updated: {updated} chapters")
    print(f"Skipped: {skipped} chapters (already had toolbar)")
    print(f"{'='*50}")

if __name__ == "__main__":
    main()
