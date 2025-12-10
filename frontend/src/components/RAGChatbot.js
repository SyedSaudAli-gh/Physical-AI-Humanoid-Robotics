import React, { useState, useRef, useEffect } from 'react';
import Layout from '@theme/Layout';
import { fetchChapterContent } from '../services/api';
import config from '../services/config';

// RAG Chatbot component
const RAGChatbot = ({ chapterId }) => {
  const [messages, setMessages] = useState([]);
  const [inputText, setInputText] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [selectedText, setSelectedText] = useState('');
  const [chapterContent, setChapterContent] = useState('');
  
  const messagesEndRef = useRef(null);
  const textareaRef = useRef(null);

  // Load chapter content if chapterId is provided
  useEffect(() => {
    if (chapterId) {
      loadChapterContent(chapterId);
    }
  }, [chapterId]);

  const loadChapterContent = async (id) => {
    try {
      const content = await fetchChapterContent(id);
      setChapterContent(content.content || '');
    } catch (error) {
      console.error('Error loading chapter content:', error);
    }
  };

  // Function to handle text selection
  useEffect(() => {
    const handleSelection = () => {
      const selectedText = window.getSelection().toString().trim();
      if (selectedText) {
        setSelectedText(selectedText);
      }
    };

    document.addEventListener('mouseup', handleSelection);
    return () => {
      document.removeEventListener('mouseup', handleSelection);
    };
  }, []);

  // Scroll to bottom of messages
  useEffect(() => {
    scrollToBottom();
  }, [messages]);

  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  };

  const handleSendMessage = async () => {
    if (!inputText.trim() || isLoading) return;

    // Add user message to chat
    const userMessage = { type: 'user', content: inputText, timestamp: new Date() };
    setMessages(prev => [...prev, userMessage]);
    
    const currentInput = inputText;
    const currentSelectedText = selectedText;
    setInputText('');
    setIsLoading(true);

    try {
      // Call the backend API
      const apiUrl = config.getApiBaseUrl();
      const response = await fetch(`${apiUrl}/api/chat/query`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          query: currentInput,
          selected_text: currentSelectedText,
          chapter_id: chapterId,
          include_context: true
        })
      });

      if (!response.ok) {
        throw new Error(`HTTP error! status: ${response.status}`);
      }

      const data = await response.json();
      
      // Add bot response to chat
      const botMessage = { 
        type: 'bot', 
        content: data.response, 
        sources: data.sources,
        timestamp: new Date() 
      };
      setMessages(prev => [...prev, botMessage]);
    } catch (error) {
      console.error('Error sending message:', error);
      const errorMessage = { 
        type: 'bot', 
        content: 'Sorry, I encountered an error processing your request. Please try again.', 
        timestamp: new Date() 
      };
      setMessages(prev => [...prev, errorMessage]);
    } finally {
      setIsLoading(false);
      setSelectedText(''); // Clear selected text after sending
    }
  };

  const handleKeyPress = (e) => {
    if (e.key === 'Enter' && !e.shiftKey) {
      e.preventDefault();
      handleSendMessage();
    }
  };

  const handleSuggestionClick = (suggestion) => {
    setInputText(suggestion);
    textareaRef.current?.focus();
  };

  // Chat suggestions to help users get started
  const suggestions = [
    "Explain the key concepts in this chapter",
    "How does this relate to real-world robotics?",
    "Show me the code example again",
    "What are the learning outcomes of this chapter?"
  ];

  return (
    <Layout title="RAG Chatbot">
      <div className="container margin-vert--lg">
        <div className="row">
          <div className="col col--12">
            <h2>Textbook Assistant</h2>
            <p>Ask questions about the textbook content, and I'll help you find relevant information.</p>
            
            {selectedText && (
              <div className="alert alert--info margin-vert--md">
                <strong>Selected text:</strong> "{selectedText.substring(0, 100)}{selectedText.length > 100 ? '...' : ''}"
              </div>
            )}
            
            <div className="chat-container">
              <div className="chat-messages" style={{ height: '400px', overflowY: 'auto', border: '1px solid #ddd', padding: '1rem', borderRadius: '4px', marginBottom: '1rem' }}>
                {messages.length === 0 ? (
                  <div className="text--center padding--md">
                    <p>Ask me anything about the textbook content!</p>
                    <p>I can help explain concepts, find relevant information, and answer questions.</p>
                  </div>
                ) : (
                  messages.map((message, index) => (
                    <div key={index} className={`chat-message ${message.type}`} style={{ marginBottom: '1rem' }}>
                      <div className={`message-bubble ${message.type === 'user' ? 'user-message' : 'bot-message'}`} 
                           style={{ 
                             textAlign: message.type === 'user' ? 'right' : 'left',
                             marginLeft: message.type === 'user' ? 'auto' : '0',
                             maxWidth: '80%',
                             padding: '0.75rem',
                             borderRadius: '8px',
                             backgroundColor: message.type === 'user' ? '#e3f2fd' : '#f5f5f5'
                           }}>
                        <div style={{ fontWeight: 'bold', marginBottom: '0.25rem' }}>
                          {message.type === 'user' ? 'You' : 'Textbook Assistant'}
                        </div>
                        <div>{message.content}</div>
                        {message.sources && message.sources.length > 0 && (
                          <div style={{ marginTop: '0.5rem', fontSize: '0.85em', color: '#666' }}>
                            Sources: {message.sources.map((src, idx) => (
                              <span key={idx} className="badge badge--secondary" style={{ marginRight: '0.25rem' }}>
                                {src.chapter_id || 'Chapter'}
                              </span>
                            ))}
                          </div>
                        )}
                      </div>
                    </div>
                  ))
                )}
                {isLoading && (
                  <div className="chat-message bot" style={{ marginBottom: '1rem' }}>
                    <div 
                      className="message-bubble bot-message" 
                      style={{ 
                        marginLeft: 0,
                        maxWidth: '80%',
                        padding: '0.75rem',
                        borderRadius: '8px',
                        backgroundColor: '#f5f5f5'
                      }}
                    >
                      <div style={{ fontWeight: 'bold', marginBottom: '0.25rem' }}>Textbook Assistant</div>
                      <div>Thinking...</div>
                    </div>
                  </div>
                )}
                <div ref={messagesEndRef} />
              </div>
              
              {messages.length === 0 && (
                <div className="margin-vert--md">
                  <h4>Try asking:</h4>
                  <div className="row">
                    {suggestions.map((suggestion, index) => (
                      <div key={index} className="col col--6 margin-vert--sm">
                        <button 
                          className="button button--outline button--secondary"
                          onClick={() => handleSuggestionClick(suggestion)}
                          style={{ width: '100%', height: '100%', textAlign: 'left' }}
                        >
                          {suggestion}
                        </button>
                      </div>
                    ))}
                  </div>
                </div>
              )}
              
              <div className="chat-input-area">
                <div style={{ display: 'flex', gap: '0.5rem' }}>
                  <textarea
                    ref={textareaRef}
                    value={inputText}
                    onChange={(e) => setInputText(e.target.value)}
                    onKeyPress={handleKeyPress}
                    placeholder="Ask a question about the textbook content..."
                    style={{ 
                      flex: 1, 
                      padding: '0.75rem', 
                      borderRadius: '4px', 
                      border: '1px solid #ddd',
                      minHeight: '60px',
                      resize: 'vertical'
                    }}
                    disabled={isLoading}
                  />
                  <button 
                    className="button button--primary"
                    onClick={handleSendMessage}
                    disabled={!inputText.trim() || isLoading}
                    style={{ alignSelf: 'flex-start', height: 'fit-content', padding: '0.75rem 1.5rem' }}
                  >
                    {isLoading ? 'Sending...' : 'Send'}
                  </button>
                </div>
              </div>
            </div>
          </div>
        </div>
      </div>
    </Layout>
  );
};

export default RAGChatbot;