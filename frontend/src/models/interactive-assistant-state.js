/**
 * Model representing the state of the interactive assistant
 * @typedef {Object} InteractiveAssistantState
 * @property {boolean} isOpen - Tracks whether the chat interface is expanded
 * @property {boolean} isProcessing - Indicates if a query is currently being processed
 * @property {string} [selectedText] - Stores text selected by the user for contextual queries
 * @property {Array<ChatMessage>} messages - Stores the conversation history
 * @property {string} [error] - Stores any error messages from the chat interface
 */

/**
 * Creates a new InteractiveAssistantState instance
 * @param {Object} params - Parameters for creating an InteractiveAssistantState
 * @param {boolean} params.isOpen - Tracks whether the chat interface is expanded
 * @param {boolean} params.isProcessing - Indicates if a query is currently being processed
 * @param {string} [params.selectedText] - Stores text selected by the user for contextual queries
 * @param {Array<ChatMessage>} params.messages - Stores the conversation history
 * @param {string} [params.error] - Stores any error messages from the chat interface
 * @returns {InteractiveAssistantState} A new InteractiveAssistantState instance
 */
export const createInteractiveAssistantState = ({ 
  isOpen = false, 
  isProcessing = false, 
  selectedText = '', 
  messages = [], 
  error = null 
}) => {
  return {
    isOpen,
    isProcessing,
    selectedText,
    messages,
    error
  };
};

/**
 * Validates if an object conforms to the InteractiveAssistantState structure
 * @param {any} obj - Object to validate
 * @returns {boolean} True if the object is a valid InteractiveAssistantState
 */
export const isValidInteractiveAssistantState = (obj) => {
  return (
    typeof obj === 'object' &&
    obj !== null &&
    typeof obj.isOpen === 'boolean' &&
    typeof obj.isProcessing === 'boolean' &&
    (typeof obj.selectedText === 'string' || obj.selectedText === undefined || obj.selectedText === null) &&
    Array.isArray(obj.messages) &&
    (typeof obj.error === 'string' || obj.error === null || obj.error === undefined)
  );
};

export default { createInteractiveAssistantState, isValidInteractiveAssistantState };