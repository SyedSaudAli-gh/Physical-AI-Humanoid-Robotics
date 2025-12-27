// src/theme/Root.js
import React, { useState, useEffect } from 'react';
import { UserProvider } from '../contexts/UserContext';
import { AuthProvider } from '../contexts/AuthContext';
import ExecutionEnvironment from '@docusaurus/ExecutionEnvironment';

// Default implementation, that you can customize
function Root({ children }) {
  const [selectedText, setSelectedText] = useState('');

  // Function to handle text selection
  useEffect(() => {
    // Only run on browser (not during SSR)
    if (!ExecutionEnvironment.canUseDOM) {
      return;
    }

    const handleTextSelection = () => {
      const selection = window.getSelection();
      if (selection.toString().trim() !== '') {
        setSelectedText(selection.toString());
      } else {
        setSelectedText('');
      }
    };

    // Add event listeners for text selection
    document.addEventListener('mouseup', handleTextSelection);
    document.addEventListener('keyup', handleTextSelection);

    // Cleanup event listeners on component unmount
    return () => {
      document.removeEventListener('mouseup', handleTextSelection);
      document.removeEventListener('keyup', handleTextSelection);
    };
  }, []);

  console.log("Features loaded"); // Confirmation that new code is running

  return (
    <AuthProvider>
      <UserProvider>
        {children}
      </UserProvider>
    </AuthProvider>
  );
}

export default Root;