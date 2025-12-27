/**
 * Test component to verify all functionality works on different viewport sizes
 */

import React, { useState } from 'react';
import Header from '../Header/Header';
import HeroSection from '../HeroSection/HeroSection';
import Footer from '../Footer/Footer';
import BookLayout from '../BookLayout';
import ChatWidget from '../ChatWidget/ChatWidget';

const TestAllFunctionality = () => {
  const [selectedText, setSelectedText] = useState('');
  
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
      
      <p>
        To test text selection functionality, select any text in this paragraph and then 
        open the chat widget to see how it handles selected text context.
      </p>
    </>
  );

  return (
    <div>
      <Header />
      
      <HeroSection 
        modules={modules} 
        heroImage="/img/hero/hero-image-placeholder.svg"
      />
      
      <BookLayout 
        title="Physical AI & Humanoid Robotics" 
        authors={["S. Ali", "Contributing Authors"]}
        section="Module 1: Foundations"
      >
        {sampleChapterContent}
      </BookLayout>
      
      <Footer />
      
      <ChatWidget />
      
      <div style={{ padding: '20px', marginTop: '40px', borderTop: '1px solid #eee' }}>
        <h2>Functionality Verification</h2>
        <p>Test the following on different viewport sizes:</p>
        <ul>
          <li>Header navigation and hamburger menu</li>
          <li>Hero section with module cards</li>
          <li>Book-style layout and typography</li>
          <li>Footer links and responsiveness</li>
          <li>Chat widget open/close functionality</li>
          <li>Text selection and context passing to chat</li>
          <li>All interactive elements (buttons, links, inputs)</li>
        </ul>
        
        <div style={{ marginTop: '20px' }}>
          <h3>Viewport Size Indicator</h3>
          <p>Current viewport: {window.innerWidth}px wide</p>
          <p>Current breakpoint: 
            {window.innerWidth < 768 ? 'Mobile' : 
             window.innerWidth < 1024 ? 'Tablet' : 'Desktop'}
          </p>
        </div>
      </div>
    </div>
  );
};

export default TestAllFunctionality;