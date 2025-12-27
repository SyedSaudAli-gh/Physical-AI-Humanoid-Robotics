/**
 * Test component to verify UI elements across different modules and chapters
 * This component demonstrates the book-style layout with various content types
 */

import React from 'react';
import BookLayout from '../components/BookLayout';

const TestBookUI = () => {
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
      
      <blockquote>
        "The future of AI lies not in abstract computation, but in systems that can 
        understand and interact with the physical world as naturally as humans do."
      </blockquote>
      
      <h3>Humanoid Robotics</h3>
      
      <p>
        Humanoid robots represent one of the most challenging and promising areas of 
        Physical AI. These systems aim to replicate human form and function, enabling 
        more intuitive human-robot interaction and the application of human-centered 
        environments to robotic systems.
      </p>
      
      <p>
        The development of humanoid robots requires integration of multiple disciplines 
        including mechanical engineering, control systems, computer vision, natural 
        language processing, and cognitive architectures.
      </p>
    </>
  );

  return (
    <BookLayout 
      title="Physical AI & Humanoid Robotics" 
      authors={["S. Ali", "Contributing Authors"]}
      section="Module 1: Foundations"
    >
      {sampleChapterContent}
    </BookLayout>
  );
};

export default TestBookUI;