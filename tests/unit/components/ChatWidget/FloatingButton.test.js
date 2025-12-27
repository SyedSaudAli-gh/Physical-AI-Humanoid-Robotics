import React from 'react';
import { render, screen, fireEvent } from '@testing-library/react';
import '@testing-library/jest-dom';
import FloatingButton from '../../../../frontend/src/components/ChatWidget/FloatingButton';

describe('FloatingButton', () => {
  test('renders correctly', () => {
    const mockOnClick = jest.fn();
    
    render(<FloatingButton onClick={mockOnClick} />);
    
    const button = screen.getByRole('button', { name: /Open chat assistant/i });
    expect(button).toBeInTheDocument();
    
    fireEvent.click(button);
    expect(mockOnClick).toHaveBeenCalledTimes(1);
  });
});