/**
 * HeroSection component for the Physical AI & Humanoid Robotics textbook
 * Features module navigation cards, AI-generated hero image, and clear call-to-action
 */

import React from 'react';
import Link from '@docusaurus/Link';
import ModuleCard from './ModuleCard';
import './HeroSection.css';

const HeroSection = () => {
  const modules = [
    {
      id: 'ros2',
      title: 'ROS 2 Fundamentals',
      description: 'Learn the basics of Robot Operating System 2 for humanoid robotics applications',
      icon: 'ros-icon',
      url: '/docs/ros2/intro',
      chapterCount: 4,
    },
    {
      id: 'simulation',
      title: 'Gazebo & Unity Simulation',
      description: 'Master physics simulation environments for humanoid robot behaviors',
      icon: 'simulation-icon',
      url: '/docs/simulation/intro',
      chapterCount: 3,
    },
    {
      id: 'isaac',
      title: 'NVIDIA Isaac Navigation',
      description: 'Advanced navigation using NVIDIA Isaac robotics platform',
      icon: 'navigation-icon',
      url: '/docs/isaac/intro',
      chapterCount: 4,
    },
    {
      id: 'vla',
      title: 'Vision-Language-Action',
      description: 'Multimodal AI connecting vision, language, and robotic action',
      icon: 'vla-icon',
      url: '/docs/vla/intro',
      chapterCount: 3,
    },
  ];

  return (
    <section className="hero-section" role="banner">
      <div className="hero-container">

        <div className="hero-content">
          <h1 className="hero-title">Physical AI & Humanoid Robotics</h1>

          <p className="hero-subtitle" id="hero-description">
            Bridging Digital Minds to Physical Bodies
          </p>

          <div className="hero-image-container">
            <img
              src="img/hero/hero-physical-ai-humanoid.png"
              alt="Physical AI & Humanoid Robotics"
              className="hero-image"
            />
          </div>

          {/* ✅ FIXED CTA */}
          <div className="cta-container">
            <Link
              to="/docs/ros2/intro"
              className="cta-button"
              aria-describedby="hero-description"
            >
              Explore Modules
            </Link>
          </div>
        </div>

        {/* ✅ CLICKABLE MODULES */}
        <div className="module-grid">
          <h2 className="module-grid-title">Course Modules</h2>

          <div className="module-cards-container">
            {modules.map((module) => (
              <ModuleCard
                key={module.id}
                title={module.title}
                description={module.description}
                icon={module.icon}
                url={module.url}  
                chapterCount={module.chapterCount}
              />
            ))}
          </div>
        </div>

      </div>
    </section>
  );
};

export default HeroSection;
