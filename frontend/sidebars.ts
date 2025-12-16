import type { SidebarsConfig } from '@docusaurus/plugin-content-docs';

/**
 * Creating a sidebar enables you to:
 - create an ordered group of docs
 - render a sidebar for each doc of that group
 - provide next/previous navigation

 The sidebars can be generated from the filesystem, or explicitly defined here.

 Create as many sidebars as you want.
 */
const sidebars: SidebarsConfig = {
  // Course sidebar with all modules and chapters
  courseSidebar: [
    {
      type: 'doc',
      id: 'intro',
      label: 'Introduction to Physical AI',
    },
    {
      type: 'category',
      label: 'Module 1: The Robotic Nervous System (ROS 2)',
      collapsed: false,
      items: [
        'module-1/chapter-1',
        'module-1/chapter-2',
        'module-1/chapter-3',
        'module-1/chapter-4',
        'module-1/chapter-5',
      ],
    },
    {
      type: 'category',
      label: 'Module 2: The Digital Twin (Gazebo & Unity)',
      collapsed: false,
      items: [
        'module-2/chapter-6',
        'module-2/chapter-7',
        'module-2/chapter-8',
        'module-2/chapter-9',
      ],
    },
    {
      type: 'category',
      label: 'Module 3: The AI-Robot Brain (NVIDIA Isaac™)',
      collapsed: false,
      items: [
        'module-3/chapter-10',
        'module-3/chapter-11',
        'module-3/chapter-12',
        'module-3/chapter-13',
      ],
    },
    {
      type: 'category',
      label: 'Module 4: Vision-Language-Action (VLA)',
      collapsed: false,
      items: [
        'module-4/chapter-14',
        'module-4/chapter-15',
        'module-4/chapter-16',
        'module-4/chapter-17',
      ],
    },
    {
      type: 'category',
      label: 'Resources',
      collapsed: true,
      items: [
        'hardware',
        'lab-setup',
        'assessments',
      ],
    },
  ],
};

export default sidebars;
