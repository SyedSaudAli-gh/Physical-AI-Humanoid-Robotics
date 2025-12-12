import React, { useState, useEffect, useRef } from 'react';
import axios from 'axios';

const RAGChatbot = ({ selectedText = null }) => {
  const [isOpen, setIsOpen] = useState(false);
  const [messages, setMessages] = useState([]);
  const [inputValue, setInputValue] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [selectedTextContext, setSelectedTextContext] = useState(selectedText);
  const messagesEndRef = useRef(null);

  // Update selected text context when it changes
  useEffect(() => {
    setSelectedTextContext(selectedText);
  }, [selectedText]);

  // Scroll to bottom of messages when they change
  useEffect(() => {
    scrollToBottom();
  }, [messages, isLoading]);

  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  };

  const toggleChatbot = () => {
    setIsOpen(!isOpen);
  };

  const clearChat = () => {
    setMessages([]);
  };

  const sendMessage = async () => {
    if (!inputValue.trim()) return;

    // Add user message to the chat
    const userMessage = {
      id: Date.now(),
      role: 'user',
      content: inputValue,
      timestamp: new Date().toISOString()
    };
    setMessages(prev => [...prev, userMessage]);
    setInputValue('');
    setIsLoading(true);

    try {
      // Prepare the request to the backend
      const response = await axios.post('/api/chat/query', {
        query: inputValue,
        selected_text: selectedTextContext
      });

      // Add assistant response to the chat
      const botMessage = {
        id: Date.now() + 1,
        role: 'assistant',
        content: response.data.answer,
        sources: response.data.sources,
        timestamp: new Date().toISOString()
      };

      setMessages(prev => [...prev, botMessage]);
    } catch (error) {
      console.error('Error sending message:', error);
      const errorMessage = {
        id: Date.now() + 1,
        role: 'assistant',
        content: 'Sorry, I encountered an error processing your request. Please try again.',
        timestamp: new Date().toISOString()
      };
      setMessages(prev => [...prev, errorMessage]);
    } finally {
      setIsLoading(false);
    }
  };

  const handleKeyDown = (e) => {
    if (e.key === 'Enter' && !e.shiftKey) {
      e.preventDefault();
      sendMessage();
    }
  };

  return (
    <div className={`rag-chatbot ${isOpen ? 'is-open' : ''}`}>
      {/* Chatbot button */}
      <button
        className="chatbot-button"
        onClick={toggleChatbot}
        aria-label={isOpen ? "Close chatbot" : "Open chatbot"}
        title={isOpen ? "Close chatbot" : "Open chatbot"}
      >
        {isOpen ? (
          <svg xmlns="http://www.w3.org/2000/svg" width="24" height="24" fill="currentColor" viewBox="0 0 16 16">
            <path d="M4.646 4.646a.5.5 0 0 1 .708 0L8 7.293l2.646-2.647a.5.5 0 0 1 .708.708L8.707 8l2.647 2.646a.5.5 0 0 1-.708.708L8 8.707l-2.646 2.647a.5.5 0 0 1-.708-.708L7.293 8 4.646 5.354a.5.5 0 0 1 0-.708z"/>
          </svg>
        ) : (
          <svg xmlns="http://www.w3.org/2000/svg" width="24" height="24" fill="currentColor" viewBox="0 0 16 16">
            <path d="M6 12.5a.5.5 0 0 1 .5-.5h9a.5.5 0 0 1 0 1h-9a.5.5 0 0 1-.5-.5z"/>
            <path d="M.5 1a.5.5 0 0 0 0 1h1.586a.5.5 0 0 1 .354.146l1.414 1.414a.5.5 0 0 0 .354.146h1.293a.5.5 0 0 1 .354.146l1.414 1.414a.5.5 0 0 0 .354.146h1.293a.5.5 0 0 1 .354.146l1.414 1.414a.5.5 0 0 0 .354.146h1.293a.5.5 0 0 1 .354.146l.5.5V.5a.5.5 0 0 0-.5-.5H.5z"/>
          </svg>
        )}
      </button>

      {/* Chatbot window */}
      {isOpen && (
        <div className="chatbot-window">
          <div className="chatbot-header">
            <div className="chatbot-header-content">
              <h4>
                <i className="fas fa-robot"></i> Textbook Assistant
              </h4>
              <div className="chatbot-header-actions">
                <button className="chatbot-action-button" onClick={clearChat} title="Clear chat">
                  <i className="fas fa-trash-alt"></i>
                </button>
                <button
                  className="chatbot-action-button close-button"
                  onClick={toggleChatbot}
                  title="Close"
                >
                  <i className="fas fa-times"></i>
                </button>
              </div>
            </div>
          </div>

          <div className="chatbot-messages-container">
            {messages.length === 0 ? (
              <div className="chatbot-welcome">
                <div className="welcome-icon">
                  <i className="fas fa-robot"></i>
                </div>
                <h5>Hello there! 👋</h5>
                <p>I'm your Physical AI & Humanoid Robotics textbook assistant.</p>
                <p>Ask me questions about the content, or use text I found selected on the page:</p>
                {selectedTextContext && (
                  <div className="selected-text-preview">
                    <p><strong>Selected text:</strong> "{selectedTextContext.substring(0, 100)}{selectedTextContext.length > 100 ? '...' : ''}"</p>
                  </div>
                )}
              </div>
            ) : (
              <div className="chatbot-messages">
                {messages.map((message) => (
                  <div key={message.id} className={`message message--${message.role}`}>
                    <div className="message-avatar">
                      {message.role === 'user' ? (
                        <i className="fas fa-user"></i>
                      ) : (
                        <i className="fas fa-robot"></i>
                      )}
                    </div>
                    <div className="message-content-wrapper">
                      <div className="message-content">
                        {message.content}
                      </div>
                      {message.sources && message.sources.length > 0 && (
                        <div className="message-sources">
                          <details className="sources-details">
                            <summary>Sources</summary>
                            <ul>
                              {message.sources.map((source, idx) => (
                                <li key={idx}>
                                  {source.chapter_title} (Relevance: {(source.similarity_score * 100).toFixed(1)}%)
                                </li>
                              ))}
                            </ul>
                          </details>
                        </div>
                      )}
                    </div>
                  </div>
                ))}
                {isLoading && (
                  <div className="message message--assistant">
                    <div className="message-avatar">
                      <i className="fas fa-robot"></i>
                    </div>
                    <div className="message-content-wrapper">
                      <div className="typing-indicator">
                        <span></span>
                        <span></span>
                        <span></span>
                      </div>
                    </div>
                  </div>
                )}
                <div ref={messagesEndRef} />
              </div>
            )}
          </div>

          <div className="chatbot-input-container">
            {selectedTextContext && (
              <div className="selected-text-indicator">
                <i className="fas fa-quote-left"></i>
                <span>{selectedTextContext.substring(0, 70)}{selectedTextContext.length > 70 ? '...' : ''}</span>
              </div>
            )}
            <div className="chatbot-input">
              <textarea
                value={inputValue}
                onChange={(e) => setInputValue(e.target.value)}
                onKeyDown={handleKeyDown}
                placeholder="Ask a question about the textbook..."
                rows="1"
                autoFocus
              />
              <button
                onClick={sendMessage}
                disabled={isLoading || !inputValue.trim()}
                className="send-button"
              >
                <i className="fas fa-paper-plane"></i>
              </button>
            </div>
          </div>
        </div>
      )}
    </div>
  );
};

export default RAGChatbot;