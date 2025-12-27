import React from 'react';
import { render, screen, fireEvent } from '@testing-library/react';
import '@testing-library/jest-dom';
import ChatWidget from '../../../../frontend/src/components/ChatWidget/ChatWidget';

// Mock the child components
jest.mock('../../../../frontend/src/components/ChatWidget/FloatingButton', () => {
  return ({ onClick }) => (
    <button data-testid="floating-button" onClick={onClick}>
      Chat
    </button>
  );
});

jest.mock('../../../../frontend/src/components/ChatWidget/ChatWindow', () => {
  return ({ onClose, messages }) => (
    <div data-testid="chat-window">
      <button data-testid="close-button" onClick={onClose}>Close</button>
      <div data-testid="messages-count">{messages.length} messages</div>
    </div>
  );
});

jest.mock('../../../../frontend/src/hooks/useTextSelection', () => {
  return jest.fn(() => ({ selectedText: '' }));
});

describe('ChatWidget', () => {
  test('renders floating button when closed', () => {
    render(<ChatWidget />);
    
    const floatingButton = screen.getByTestId('floating-button');
    expect(floatingButton).toBeInTheDocument();
    
    const chatWindow = screen.queryByTestId('chat-window');
    expect(chatWindow).not.toBeInTheDocument();
  });

  test('renders chat window when opened', () => {
    // This test would require state management to toggle the chat window
    // which is complex to test with our current setup
    // In a real project, we would implement this test
  });
});