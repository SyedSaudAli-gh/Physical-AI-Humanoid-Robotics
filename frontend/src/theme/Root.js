// src/theme/Root.js
import React, { useState, useEffect } from 'react';
import { UserProvider } from '../contexts/UserContext';
import { AuthProvider } from '../contexts/AuthContext';
import RAGChatbot from '../components/RAGChatbot';
import AuthNavbarItem from '../components/AuthNavbarItem';

// Default implementation, that you can customize
function Root({ children }) {
  const [selectedText, setSelectedText] = useState('');

  // Function to handle text selection
  useEffect(() => {
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
        <>
          {/* Add the AuthNavbarItem as a global element */}
          <div style={{ position: 'fixed', top: '10px', right: '10px', zIndex: 1000 }}>
            <AuthNavbarItem />
          </div>
          {children}
          <RAGChatbot selectedText={selectedText} />
        </>
      </UserProvider>
    </AuthProvider>
  );
}

export default Root;