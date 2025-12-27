/**
 * Test component for chatbot functionality
 * Verifies chatbot works with various queries and contexts
 */

import React, { useState } from 'react';
import ChatWidget from '../ChatWidget/ChatWidget';

const TestChatbot = () => {
  const [selectedText, setSelectedText] = useState('');
  const [testContext, setTestContext] = useState('');

  const handleSetTestContext = (context) => {
    setTestContext(context);
    setSelectedText(context);
  };

  return (
    <div style={{ padding: '20px' }}>
      <h1>Chatbot Functionality Test</h1>
      <p>This page allows testing of the chatbot with various contexts.</p>
      
      <div style={{ marginBottom: '20px' }}>
        <h3>Test Contexts</h3>
        <button 
          onClick={() => handleSetTestContext('Physical AI represents a convergence of artificial intelligence and physical systems, where intelligent agents interact with and learn from the real world.')}
          style={{ margin: '5px', padding: '8px' }}
        >
          Set Physical AI Context
        </button>
        <button 
          onClick={() => handleSetTestContext('Humanoid robots require integration of multiple disciplines including mechanical engineering, control systems, computer vision, and cognitive architectures.')}
          style={{ margin: '5px', padding: '8px' }}
        >
          Set Humanoid Robotics Context
        </button>
        <button 
          onClick={() => handleSetTestContext('ROS 2 uses a publish-subscribe pattern for communication between nodes.')}
          style={{ margin: '5px', padding: '8px' }}
        >
          Set ROS Context
        </button>
        <button 
          onClick={() => setSelectedText('')}
          style={{ margin: '5px', padding: '8px' }}
        >
          Clear Context
        </button>
      </div>

      <div style={{ marginTop: '40px' }}>
        <h3>Chat Interface</h3>
        <p>Selected text for context: "{selectedText || 'None'}"</p>
        <ChatWidget />
      </div>
    </div>
  );
};

export default TestChatbot;