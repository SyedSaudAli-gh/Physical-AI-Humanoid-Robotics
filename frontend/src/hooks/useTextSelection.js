/**
 * Custom React hook for handling text selection functionality
 * Allows users to select text on the page and use it as context for queries
 */

import { useState, useEffect } from 'react';

/**
 * A hook that tracks selected text on the page
 * @returns {Object} An object containing the selected text and related properties
 * @property {string} selectedText - The currently selected text on the page
 */
const useTextSelection = () => {
  const [selectedText, setSelectedText] = useState('');

  useEffect(() => {
    const handleSelectionChange = () => {
      const selection = window.getSelection();
      const text = selection.toString().trim();
      
      // Only update if the selected text has actually changed
      if (text !== selectedText) {
        setSelectedText(text);
      }
    };

    // Add event listeners for selection changes
    document.addEventListener('selectionchange', handleSelectionChange);
    
    // Also listen for mouseup and keyup events as additional triggers
    // since selectionchange might not fire in all browsers in all situations
    document.addEventListener('mouseup', handleSelectionChange);
    document.addEventListener('keyup', handleSelectionChange);

    // Cleanup function to remove event listeners
    return () => {
      document.removeEventListener('selectionchange', handleSelectionChange);
      document.removeEventListener('mouseup', handleSelectionChange);
      document.removeEventListener('keyup', handleSelectionChange);
    };
  }, [selectedText]); // Include selectedText in the dependency array to avoid stale closure

  // Function to clear the current selection
  const clearSelection = () => {
    if (window.getSelection) {
      window.getSelection().removeAllRanges();
    } else if (document.selection) {
      document.selection.empty(); // For older IE versions
    }
    setSelectedText('');
  };

  return { selectedText, clearSelection };
};

export default useTextSelection;