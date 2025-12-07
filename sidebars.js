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
  // Main tutorial sidebar
  tutorialSidebar: [
    {
      type: 'doc',
      id: 'intro',
      label: 'Introduction',
    },
    {
      type: 'doc',
      id: 'quick-start',
      label: 'Quick Start',
    },
    {
      type: 'category',
      label: 'Module 1: ROS 2 Fundamentals',
      link: {
        type: 'doc',
        id: 'module-1-ros2/index',
      },
      items: [
        'module-1-ros2/ros2-basics-first-node',
        'module-1-ros2/topics-services',
        'module-1-ros2/urdf-humanoids',
      ],
    },
    {
      type: 'category',
      label: 'Module 2: Gazebo Simulation',
      link: {
        type: 'doc',
        id: 'module-2-gazebo/module-2-gazebo',
      },
      items: [
        'module-2-gazebo/simulation-essentials',
        'module-2-gazebo/sensors-ros2',
      ],
    },
    {
      type: 'category',
      label: 'Module 3: NVIDIA Isaac',
      link: {
        type: 'doc',
        id: 'module-3-isaac/module-3-isaac',
      },
      items: [
        'module-3-isaac/isaac-sim-essentials',
        'module-3-isaac/synthetic-data-perception',
        'module-3-isaac/vslam',
        'module-3-isaac/navigation-humanoids',
      ],
    },
    {
      type: 'category',
      label: 'Module 4: Vision-Language-Action',
      link: {
        type: 'doc',
        id: 'module-4-vla/module-4-vla',
      },
      items: [
        'module-4-vla/vla-voice-to-action',
        'module-4-vla/llm-planning',
        'module-4-vla/integration-case-studies',
      ],
    },
    {
      type: 'doc',
      id: 'glossary',
      label: 'Glossary',
    },
  ],
};

module.exports = sidebars;
