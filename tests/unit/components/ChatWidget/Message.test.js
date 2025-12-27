import React from 'react';
import { render, screen } from '@testing-library/react';
import '@testing-library/jest-dom';
import Message from '../../../../frontend/src/components/ChatWidget/Message';

describe('Message', () => {
  test('renders user message correctly', () => {
    render(<Message text="Hello" isUser={true} />);
    
    const messageElement = screen.getByText('Hello');
    expect(messageElement).toBeInTheDocument();
    expect(messageElement).toHaveClass('message', 'user');
  });

  test('renders bot message correctly', () => {
    render(<Message text="Hello" isUser={false} />);
    
    const messageElement = screen.getByText('Hello');
    expect(messageElement).toBeInTheDocument();
    expect(messageElement).toHaveClass('message', 'bot');
  });

  test('renders sources when provided', () => {
    const sources = [
      { title: 'Source 1', url: 'http://example.com/1' },
      { title: 'Source 2', url: 'http://example.com/2' }
    ];
    
    render(<Message text="Hello" isUser={false} sources={sources} />);
    
    const sourceLink1 = screen.getByText('Source 1');
    const sourceLink2 = screen.getByText('Source 2');
    
    expect(sourceLink1).toBeInTheDocument();
    expect(sourceLink2).toBeInTheDocument();
    expect(sourceLink1).toHaveAttribute('href', 'http://example.com/1');
    expect(sourceLink2).toHaveAttribute('href', 'http://example.com/2');
  });
});