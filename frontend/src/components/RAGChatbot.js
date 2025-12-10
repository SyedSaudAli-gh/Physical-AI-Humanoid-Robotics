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
  }, [messages]);

  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  };

  const toggleChatbot = () => {
    setIsOpen(!isOpen);
  };

  const sendMessage = async () => {
    if (!inputValue.trim()) return;

    // Add user message to the chat
    const userMessage = { role: 'user', content: inputValue };
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
        role: 'assistant', 
        content: response.data.answer,
        sources: response.data.sources
      };
      
      setMessages(prev => [...prev, botMessage]);
    } catch (error) {
      console.error('Error sending message:', error);
      const errorMessage = { 
        role: 'assistant', 
        content: 'Sorry, I encountered an error processing your request. Please try again.' 
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
    <div className={`rag-chatbot ${isOpen ? 'open' : ''}`}>
      {/* Chatbot button */}
      <button 
        className="chatbot-button" 
        onClick={toggleChatbot}
        aria-label="Open chatbot"
      >
        <svg xmlns="http://www.w3.org/2000/svg" width="24" height="24" fill="currentColor" viewBox="0 0 16 16">
          <path d="M6 12.5a.5.5 0 0 1 .5-.5h9a.5.5 0 0 1 0 1h-9a.5.5 0 0 1-.5-.5z"/>
          <path d="M.5 1a.5.5 0 0 0 0 1h1.586a.5.5 0 0 1 .354.146l1.414 1.414a.5.5 0 0 0 .354.146h1.293a.5.5 0 0 1 .354.146l1.414 1.414a.5.5 0 0 0 .354.146h1.293a.5.5 0 0 1 .354.146l1.414 1.414a.5.5 0 0 0 .354.146h1.293a.5.5 0 0 1 .354.146l.5.5V.5a.5.5 0 0 0-.5-.5H.5z"/>
        </svg>
      </button>

      {/* Chatbot window */}
      {isOpen && (
        <div className="chatbot-window">
          <div className="chatbot-header">
            <h4>Textbook Assistant</h4>
            <button className="close-button" onClick={toggleChatbot}>
              &times;
            </button>
          </div>
          
          <div className="chatbot-messages">
            {messages.length === 0 ? (
              <div className="welcome-message">
                <p>Hello! I'm your Physical AI & Humanoid Robotics textbook assistant.</p>
                <p>Ask me questions about the content, or use text I found selected on the page:</p>
                {selectedTextContext && (
                  <div className="selected-text-preview">
                    <p><strong>Selected text:</strong> "{selectedTextContext.substring(0, 100)}{selectedTextContext.length > 100 ? '...' : ''}"</p>
                  </div>
                )}
              </div>
            ) : (
              messages.map((message, index) => (
                <div key={index} className={`message ${message.role}`}>
                  <div className="message-content">
                    {message.content}
                  </div>
                  {message.sources && message.sources.length > 0 && (
                    <div className="sources">
                      <p><small>Sources:</small></p>
                      <ul>
                        {message.sources.map((source, idx) => (
                          <li key={idx}>
                            <small>
                              {source.chapter_title} (Score: {source.similarity_score.toFixed(2)})
                            </small>
                          </li>
                        ))}
                      </ul>
                    </div>
                  )}
                </div>
              ))
            )}
            {isLoading && (
              <div className="message assistant">
                <div className="typing-indicator">
                  <span></span>
                  <span></span>
                  <span></span>
                </div>
              </div>
            )}
            <div ref={messagesEndRef} />
          </div>
          
          <div className="chatbot-input">
            <textarea
              value={inputValue}
              onChange={(e) => setInputValue(e.target.value)}
              onKeyDown={handleKeyDown}
              placeholder="Ask a question about the textbook..."
              rows="2"
            />
            <button onClick={sendMessage} disabled={isLoading}>
              Send
            </button>
          </div>
        </div>
      )}
    </div>
  );
};

export default RAGChatbot;