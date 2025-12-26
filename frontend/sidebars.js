/**
 * Creating a sidebar enables you to:
 - create an ordered group of docs
 - render a sidebar for each doc of that group
 - provide next/previous navigation

 The sidebars can be generated from the filesystem, or explicitly defined here.

 Create as many sidebars as you want.
 */

// @ts-check

/** @type {import('@docusaurus/plugin-content-docs').SidebarsConfig} */
const sidebars = {
  tutorialSidebar: [
    {
      type: 'category',
      label: 'Introduction & Setup',
      link: {
        type: 'generated-index',
        title: 'Introduction & Setup',
        description: 'Get started with the textbook and prepare your development environment.',
        slug: '/introduction-setup',
      },
      items: [
        'introduction/foundations-week1',
        'introduction/foundations-week2',
        'introduction/foundations-week3',
        {
          type: 'category',
          label: 'Setup Guides',
          link: {
            type: 'generated-index',
            title: 'Setup Guides',
            description: 'Guides for setting up your development environment.',
            slug: '/setup-guides',
          },
          items: [
            'setup/setup-workstation',
            'setup/setup-edge-kit',
            'setup/setup-cloud-native',
          ],
        },
      ],
    },
    {
      type: 'category',
      label: 'Module 1: ROS 2',
      link: {
        type: 'generated-index',
        title: 'Module 1: ROS 2',
        description: 'Learn the fundamentals of the Robot Operating System 2.',
        slug: '/ros2-module',
      },
      items: [
        'ros2/ros2-week4',
        'ros2/ros2-week5',
        'ros2/tf2',
        'ros2/urdf',
      ],
    },
    {
      type: 'category',
      label: 'Module 2: Digital Twin',
      link: {
        type: 'generated-index',
        title: 'Module 2: Digital Twin',
        description: 'Simulate robots and environments with NVIDIA Isaac Sim.',
        slug: '/digital-twin-module',
      },
      items: [
        'digital-twin/week6',
        'digital-twin/week7',
        'digital-twin/ros2-control',
      ],
    },
    {
      type: 'category',
      label: 'Module 3: NVIDIA Isaac',
      link: {
        type: 'generated-index',
        title: 'Module 3: NVIDIA Isaac',
        description: 'Accelerate robotics perception and AI with Isaac ROS.',
        slug: '/isaac-module',
      },
      items: [
        'isaac-sim/week8',
        'isaac-sim/week9',
        'isaac-sim/week10',
      ],
    },
    {
      type: 'category',
      label: 'Module 4: VLA',
      link: {
        type: 'generated-index',
        title: 'Module 4: Vision-Language-Action Models',
        description: 'Explore the cutting-edge of VLA models for humanoid robots.',
        slug: '/vla-module',
      },
      items: [
        'vla/vla-week11',
        'vla/vla-week12',
        'vla/vla-week13',
      ],
    },
    {
      type: 'category',
      label: 'Assessments',
      link: {
        type: 'generated-index',
        title: 'Assessments',
        description: 'Evaluate your knowledge and skills.',
        slug: '/assessments',
      },
      items: [
        'assessments/ros2-assessment',
        'assessments/digital-twin-assessment',
        'assessments/isaac-sim-assessment',
        'assessments/capstone-project-guide',
      ],
    },
    {
      type: 'category',
      label: 'Reference',
      link: {
        type: 'generated-index',
        title: 'Reference Materials',
        description: 'Supplementary materials and troubleshooting.',
        slug: '/reference',
      },
      items: [
        'reference/glossary',
        'reference/notation-guide',
        'reference/ros2-quick-reference',
        'reference/troubleshooting-guide',
      ],
    },
  ],
};

module.exports = sidebars;