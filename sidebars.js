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
  // By default, Docusaurus generates a sidebar from the docs folder structure
  tutorialSidebar: [
    {
      type: 'category',
      label: 'Introduction',
      items: [
        'foundations-week1',
        'foundations-week2',
        'foundations-week3',
      ],
    },
    {
      type: 'category',
      label: 'Setup',
      items: [
        'setup-workstation',
        'setup-cloud-native',
        'setup-edge-kit',
      ],
    },
    {
      type: 'category',
      label: 'Module 1: ROS 2 Foundations',
      items: [
        'week4',
        'week5',
        'tf2',
        'urdf',
      ],
    },
    {
      type: 'category',
      label: 'Module 2: Digital Twin Concepts',
      items: [
        'week6',
        'week7',
        'ros2-control',
      ],
    },
    {
      type: 'category',
      label: 'Module 3: Isaac Sim Integration',
      items: [
        'week8',
        'week9',
        'week10',
      ],
    },
  ],

  // But you can create a sidebar manually
  /*
  tutorialSidebar: [
    'introduction',
    'hello',
    {
      type: 'category',
      label: 'Tutorial',
      items: ['tutorial-basics/create-a-document'],
    },
  ],
   */
};

module.exports = sidebars;
