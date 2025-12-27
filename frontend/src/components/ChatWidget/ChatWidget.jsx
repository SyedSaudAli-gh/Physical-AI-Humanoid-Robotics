/**
 * ChatWidget Component
 */

import React, { useState, useEffect } from 'react';
import FloatingButton from './FloatingButton';
import ChatWindow from './ChatWindow';
import apiService from '../../services/api';
import useTextSelection from '../../hooks/useTextSelection';
import './styles.css';

const ChatWidget = () => {
  const [isOpen, setIsOpen] = useState(false);
  const [messages, setMessages] = useState([]);
  const [isLoading, setIsLoading] = useState(false);
  const { selectedText } = useTextSelection();

  const toggleChat = () => setIsOpen((v) => !v);

  const handleSendMessage = async (text) => {
    if (!text.trim()) return;

    // ✅ USER MESSAGE
    setMessages((prev) => [
      ...prev,
      {
        id: Date.now().toString(),
        sender: 'user',
        text: text,              // ✅ ALWAYS text
        timestamp: new Date(),
      },
    ]);

    setIsLoading(true);

    try {
      const response = await apiService.sendQuery(text, selectedText);

      setMessages((prev) => [
        ...prev,
        {
          id: (Date.now() + 1).toString(),
          sender: 'assistant',
          text: response?.answer || 'No response received.',
          timestamp: new Date(),
          sources: Array.isArray(response?.sources)
            ? response.sources
            : [],
        },
      ]);
    } catch (error) {
      setMessages((prev) => [
        ...prev,
        {
          id: (Date.now() + 1).toString(),
          sender: 'system',
          text:
            error?.message ||
            'Sorry, something went wrong. Please try again.',
          timestamp: new Date(),
          isError: true,
        },
      ]);
    } finally {
      setIsLoading(false);
    }
  };

  useEffect(() => {
    if (isOpen) {
      const el = document.querySelector('.chat-messages');
      if (el) el.scrollTop = el.scrollHeight;
    }
  }, [messages, isOpen]);

  return (
    <div className="chat-widget" role="region" aria-label="AI Assistant">
      {isOpen ? (
        <ChatWindow
          onClose={toggleChat}
          onSendMessage={handleSendMessage}
          messages={messages}
          selectedText={selectedText}
          isLoading={isLoading}
        />
      ) : (
        <FloatingButton onClick={toggleChat} />
      )}
    </div>
  );
};

export default ChatWidget;
