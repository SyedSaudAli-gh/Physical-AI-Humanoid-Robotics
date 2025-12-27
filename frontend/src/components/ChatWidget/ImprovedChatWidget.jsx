/**
 * Improved ChatWidget Component
 * Implements the interactive assistant with modern UI and light blue/black theme
 */

import React, { useState, useEffect } from 'react';
import '../ChatWidget/ImprovedChatWidget.css';

const ImprovedFloatingButton = ({ onClick }) => {
  return (
    <button
      className="improved-floating-chat-button"
      onClick={onClick}
      aria-label="Open AI Assistant"
      title="Chat with Physical AI & Humanoid Robotics textbook assistant"
      aria-expanded="false"
      role="button"
      aria-controls="improved-chat-window"
    >
      <span aria-hidden="true">💬</span>
    </button>
  );
};

const ImprovedChatWindow = ({ onClose, onSendMessage, messages = [], selectedText = '', isLoading = false }) => {
  const [inputValue, setInputValue] = useState('');
  const messagesEndRef = React.useRef(null);
  const inputRef = React.useRef(null);

  // Focus management when chat opens
  React.useEffect(() => {
    if (inputRef.current) {
      inputRef.current.focus();
    }
  }, []);

  // Auto-scroll to bottom when messages change
  React.useEffect(() => {
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
      className="improved-chat-window"
      role="dialog"
      aria-modal="true"
      aria-label="Documentation Assistant Chat"
      aria-describedby="chat-welcome"
    >
      <div className="improved-chat-header">
        <h3>AI Assistant</h3>
        <button
          className="improved-close-button"
          onClick={onClose}
          aria-label="Close chat"
        >
          ×
        </button>
      </div>

      <div className="improved-chat-messages" role="log" aria-live="polite" aria-relevant="additions text">
        {messages.length === 0 ? (
          <div className="improved-chat-welcome" id="chat-welcome" role="status" aria-live="polite">
            <p>Hello! I'm your Physical AI & Humanoid Robotics textbook assistant.</p>
            {selectedText && (
              <div className="improved-selected-text-preview">
                <p><strong>Selected text:</strong> "{selectedText.substring(0, 100)}{selectedText.length > 100 ? '...' : ''}"</p>
              </div>
            )}
            <p>Ask me anything about the content!</p>
          </div>
        ) : (
          <>
            {messages.map((message) => (
              <ImprovedChatMessage
                key={message.id}
                message={message}
                sender={message.sender === 'user' ? 'user' : message.sender === 'system' ? 'system' : 'assistant'}
              />
            ))}

            {isLoading && (
              <div className="improved-message assistant" role="status" aria-live="polite">
                <div className="improved-messageContent">
                  <div className="improved-typing-indicator" role="img" aria-label="Assistant is typing">
                    <span aria-hidden="true"></span>
                    <span aria-hidden="true"></span>
                    <span aria-hidden="true"></span>
                  </div>
                  <div className="improved-loading-text">Thinking...</div>
                </div>
              </div>
            )}
          </>
        )}
        <div ref={messagesEndRef} aria-hidden="true" />
      </div>

      <div className="improved-chat-input-area" role="form">
        {selectedText && (
          <div className="improved-selected-text-indicator" aria-label="Selected text context">
            <small>Context: "{selectedText.substring(0, 80)}{selectedText.length > 80 ? '...' : ''}"</small>
          </div>
        )}
        <div className="improved-chat-input-container">
          <textarea
            value={inputValue}
            onChange={(e) => setInputValue(e.target.value)}
            onKeyDown={handleKeyDown}
            placeholder={isLoading ? "Processing response..." : "Ask a question about the content..."}
            rows="1"
            aria-label="Type your question here"
            disabled={isLoading}
            className="improved-chat-input"
            ref={inputRef}
          />
          <button
            type="button"
            className="improved-send-button"
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

const ImprovedChatMessage = ({ message, sender }) => {
  const { text, sources, isError, timestamp } = message;

  // Format timestamp for display
  const formatTime = (date) => {
    if (!date) return '';
    const time = new Date(date);
    return time.toLocaleTimeString([], { hour: '2-digit', minute: '2-digit' });
  };

  // Generate appropriate ARIA label based on message type
  const getAriaLabel = () => {
    if (isError) {
      return `Error message: ${text}`;
    }
    const senderText = sender === 'user' ? 'Your message' : 'Assistant response';
    return `${senderText}: ${text.substring(0, 50)}${text.length > 50 ? '...' : ''}`;
  };

  return (
    <div
      className={`improved-message ${sender} ${isError ? 'error' : ''}`}
      role="listitem"
      aria-label={getAriaLabel()}
    >
      <div className="improved-messageContent">
        {isError ? (
          <div className="improved-errorMessage" role="alert" aria-live="assertive">
            <strong>Error:</strong> {text}
          </div>
        ) : (
          <div className="improved-textContent">{text}</div>
        )}

        {sources && sources.length > 0 && !isError && (
          <div className="improved-sources" role="group" aria-labelledby="sources-summary">
            <details className="improved-sourcesDetails">
              <summary id="sources-summary" aria-label="Toggle sources visibility">Sources</summary>
              <ul className="improved-sourcesList" role="list">
                {sources.map((source, index) => (
                  <li key={index} className="improved-sourceItem" role="listitem">
                    {source.title ? (
                      <a
                        href={source.url || '#'}
                        target="_blank"
                        rel="noopener noreferrer"
                        className="improved-sourceLink"
                        aria-label={`Source: ${source.title}`}
                      >
                        {source.title}
                      </a>
                    ) : (
                      <span>{source.excerpt || source}</span>
                    )}
                  </li>
                ))}
              </ul>
            </details>
          </div>
        )}
      </div>
      {timestamp && (
        <div className="improved-timestamp" aria-label={`Sent at ${formatTime(timestamp)}`}>
          {formatTime(timestamp)}
        </div>
      )}
    </div>
  );
};

const ImprovedChatWidget = () => {
  const [isOpen, setIsOpen] = useState(false);
  const [messages, setMessages] = useState([]);
  const [isLoading, setIsLoading] = useState(false);
  const selectedText = ''; // This would come from a hook in a real implementation

  const toggleChat = () => {
    setIsOpen(!isOpen);
  };

  const handleSendMessage = async (text) => {
    if (!text.trim()) return;

    // Add user message to the chat
    const userMessage = {
      id: Date.now(),
      text,
      sender: 'user',
      timestamp: new Date()
    };
    setMessages(prev => [...prev, userMessage]);

    // Set loading state
    setIsLoading(true);

    try {
      // Simulate API call delay
      await new Promise(resolve => setTimeout(resolve, 1000));

      // Add bot response to the chat
      const botMessage = {
        id: Date.now() + 1,
        text: "This is a simulated response. In the actual implementation, this would be the response from the backend API.",
        sender: 'assistant',
        sources: [
          { title: "Documentation Reference", url: "#" },
          { title: "Example Code", url: "#" }
        ],
        timestamp: new Date()
      };
      setMessages(prev => [...prev, botMessage]);
    } catch (error) {
      // Add error message to the chat
      const errorMessage = {
        id: Date.now() + 1,
        text: error.message || 'Sorry, I encountered an error. Please try again.',
        sender: 'system',
        isError: true,
        timestamp: new Date()
      };
      setMessages(prev => [...prev, errorMessage]);
    } finally {
      setIsLoading(false);
    }
  };

  // Auto-scroll to bottom when messages change
  React.useEffect(() => {
    if (isOpen) {
      const messagesContainer = document.querySelector('.improved-chat-messages');
      if (messagesContainer) {
        messagesContainer.scrollTop = messagesContainer.scrollHeight;
      }
    }
  }, [messages, isOpen]);

  // Add keyboard shortcut to toggle chat
  React.useEffect(() => {
    const handleKeyDown = (e) => {
      if ((e.ctrlKey || e.metaKey) && e.key === 'k') {
        e.preventDefault();
        toggleChat();
      }
    };

    document.addEventListener('keydown', handleKeyDown);
    return () => {
      document.removeEventListener('keydown', handleKeyDown);
    };
  }, [isOpen]);

  return (
    <div
      className="improved-chat-widget"
      aria-label="AI Assistant"
      role="region"
    >
      {isOpen ? (
        <ImprovedChatWindow
          onClose={toggleChat}
          onSendMessage={handleSendMessage}
          messages={messages}
          selectedText={selectedText}
          isLoading={isLoading}
        />
      ) : (
        <ImprovedFloatingButton onClick={toggleChat} />
      )}
    </div>
  );
};

export default ImprovedChatWidget;