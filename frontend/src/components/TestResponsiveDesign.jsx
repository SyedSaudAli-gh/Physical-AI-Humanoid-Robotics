/**
 * Test component for responsive design
 * Verifies all components are responsive across different screen sizes
 */

import React from 'react';
import Header from '../Header/Header';
import HeroSection from '../HeroSection/HeroSection';
import Footer from '../Footer/Footer';
import BookLayout from '../BookLayout';

const TestResponsiveDesign = () => {
  const modules = [
    {
      id: 'mod1',
      title: 'Foundations of Physical AI',
      description: 'Introduction to the core concepts of Physical AI and its applications.',
      chapterLink: '/docs/module1/introduction',
      icon: '🤖'
    },
    {
      id: 'mod2',
      title: 'Humanoid Robotics',
      description: 'Explore the design and control of humanoid robotic systems.',
      chapterLink: '/docs/module2/basics',
      icon: '🦾'
    }
  ];

  const sampleChapterContent = (
    <>
      <h2>Introduction to Physical AI & Humanoid Robotics</h2>
      
      <p>
        Physical AI represents a convergence of artificial intelligence and physical systems, 
        where intelligent agents interact with and learn from the real world. This field 
        encompasses robotics, embodied cognition, and adaptive systems that can perceive, 
        reason, and act in physical environments.
      </p>
      
      <h3>Key Concepts in Physical AI</h3>
      
      <p>
        The core principles of Physical AI include perception, reasoning, learning, and action. 
        These systems must be able to interpret sensory data, make decisions under uncertainty, 
        adapt to changing environments, and execute complex motor tasks.
      </p>
    </>
  );

  return (
    <div>
      <Header />
      
      <HeroSection 
        modules={modules} 
        heroImage="/img/hero/hero-image-placeholder.jpg"
      />
      
      <BookLayout 
        title="Physical AI & Humanoid Robotics" 
        authors={["S. Ali", "Contributing Authors"]}
        section="Module 1: Foundations"
      >
        {sampleChapterContent}
      </BookLayout>
      
      <Footer />
      
      <div style={{ padding: '20px', marginTop: '40px', borderTop: '1px solid #eee' }}>
        <h2>Responsive Design Test</h2>
        <p>Resize your browser window to test responsiveness across different screen sizes.</p>
        <ul>
          <li>Mobile: &lt; 768px</li>
          <li>Tablet: 768px - 1024px</li>
          <li>Desktop: &gt; 1024px</li>
        </ul>
      </div>
    </div>
  );
};

export default TestResponsiveDesign;