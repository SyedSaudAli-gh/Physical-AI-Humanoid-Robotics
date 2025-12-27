/**
 * ChatWindow Component
 * Displays the expandable chat interface with message history
 */

import React, { useState, useRef, useEffect } from 'react';
import clsx from 'clsx';
import ChatMessage from './ChatMessage';
import '../ChatWidget/styles.css'; // Main styles for the chat widget

const ChatWindow = ({ onClose, onSendMessage, messages = [], selectedText = '', isLoading = false }) => {
  const [inputValue, setInputValue] = useState('');
  const messagesEndRef = useRef(null);
  const inputRef = useRef(null);

  // Focus management when chat opens
  useEffect(() => {
    if (inputRef.current) {
      inputRef.current.focus();
    }
  }, []);

  // Auto-scroll to bottom when messages change
  useEffect(() => {
    scrollToBottom();
  }, [messages]);

  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  };

  const handleSubmit = (e) => {
    e.preventDefault();
    if (inputValue.trim() && !isLoading) {
      onSendMessage(inputValue);
      setInputValue('');
    }
  };

  const handleKeyDown = (e) => {
    if (e.key === 'Enter' && !e.shiftKey) {
      e.preventDefault();
      handleSubmit(e);
    } else if (e.key === 'Escape') {
      // Allow user to close the chat with Escape key
      onClose();
    }
  };

  return (
    <div
      className="chat-window"
      role="dialog"
      aria-modal="true"
      aria-label="Documentation Assistant Chat"
      aria-describedby="chat-welcome"
    >
      <div className="chat-header">
        <h3>AI Assistant</h3>
        <button
          className="close-button"
          onClick={onClose}
          aria-label="Close chat"
        >
          ×
        </button>
      </div>

      <div className="chat-messages" role="log" aria-live="polite" aria-relevant="additions text">
        {messages.length === 0 ? (
          <div className="chat-welcome" id="chat-welcome" role="status" aria-live="polite">
            <p>Hello! I'm your Physical AI & Humanoid Robotics textbook assistant.</p>
            {selectedText && (
              <div className="selected-text-preview">
                <p><strong>Selected text:</strong> "{selectedText.substring(0, 100)}{selectedText.length > 100 ? '...' : ''}"</p>
              </div>
            )}
            <p>Ask me anything about the content!</p>
          </div>
        ) : (
          <>
            {messages.map((message) => (
              <ChatMessage
                key={message.id}
                message={message}
                sender={message.sender === 'user' ? 'user' : message.sender === 'system' ? 'system' : 'assistant'}
              />
            ))}

            {isLoading && (
              <div className="message assistant" role="status" aria-live="polite">
                <div className="messageContent">
                  <div className="typing-indicator" role="img" aria-label="Assistant is typing">
                    <span aria-hidden="true"></span>
                    <span aria-hidden="true"></span>
                    <span aria-hidden="true"></span>
                  </div>
                  <div className="loading-text">Thinking...</div>
                </div>
              </div>
            )}
          </>
        )}
        <div ref={messagesEndRef} aria-hidden="true" />
      </div>

      <div className="chat-input-area" role="form">
        {selectedText && (
          <div className="selected-text-indicator" aria-label="Selected text context">
            <small>Context: "{selectedText.substring(0, 80)}{selectedText.length > 80 ? '...' : ''}"</small>
          </div>
        )}
        <div className="chat-input-container">
          <textarea
            value={inputValue}
            onChange={(e) => setInputValue(e.target.value)}
            onKeyDown={handleKeyDown}
            placeholder={isLoading ? "Processing response..." : "Ask a question about the content..."}
            rows="1"
            aria-label="Type your question here"
            disabled={isLoading}
            className="chat-input"
            ref={inputRef}
          />
          <button
            type="button"
            className="send-button"
            onClick={handleSubmit}
            disabled={!inputValue.trim() || isLoading}
            aria-label="Send message"
          >
            <svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 24 24" fill="currentColor">
              <path d="M3.478 2.405a.75.75 0 00-.926.94l2.432 7.905H13.5a.75.75 0 010 1.5H4.984l-2.432 7.905a.75.75 0 00.926.94 60.519 60.519 0 0018.445-8.986.75.75 0 000-1.218A60.517 60.517 0 003.478 2.405z" />
            </svg>
          </button>
        </div>
      </div>
    </div>
  );
};

export default ChatWindow;