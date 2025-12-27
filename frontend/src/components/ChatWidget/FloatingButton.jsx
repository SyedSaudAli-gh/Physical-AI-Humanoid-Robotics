import React from 'react';
import '../ChatWidget/styles.css';

const FloatingButton = ({ onClick }) => {
  return (
    <button
      className="floating-chat-button"
      onClick={onClick}
      aria-label="Open AI Assistant"
      title="Chat with Physical AI & Humanoid Robotics textbook assistant"
      aria-expanded="false"
      role="button"
      aria-controls="chat-window"
    >
      <span aria-hidden="true">💬</span>
    </button>
  );
};

export default FloatingButton;