// @ts-check

/** @type {import('@docusaurus/plugin-content-docs').SidebarsConfig} */
const sidebars = {
  tutorialSidebar: [
    'intro',
    {
      type: 'category',
      label: 'ROS 2 - The Robotic Nervous System',
      items: [
        'ros2/intro',
        'ros2/nodes',
        'ros2/topics',
        'ros2/services',
      ],
    },
    {
      type: 'category',
      label: 'Gazebo & Unity - The Digital Twin',
      items: [
        'gazebo-unity/intro',
        'gazebo-unity/simulation',
        'gazebo-unity/unity-integration',
      ],
    },
    {
      type: 'category',
      label: 'NVIDIA Isaac - AI-Driven Perception',
      items: [
        'nvidia-isaac/intro',
        'nvidia-isaac/perception',
        'nvidia-isaac/manipulation',
      ],
    },
    {
      type: 'category',
      label: 'Vision-Language-Action (VLA)',
      items: [
        'vla/intro',
        'vla/vision-language',
        'vla/action-execution',
      ],
    },
    {
      type: 'category',
      label: 'Security Best Practices',
      items: [
        'security/environment-variables',
        'security/api-security',
        'security/development-setup',
        'security/docusaurus-proxy-setup',
        'security/api-key-management-best-practices',
      ],
    },
  ],
};

module.exports = sidebars;