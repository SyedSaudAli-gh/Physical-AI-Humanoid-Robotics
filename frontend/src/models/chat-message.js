/**
 * Model representing a single chat message in the interactive assistant
 * @typedef {Object} ChatMessage
 * @property {string} id - Unique identifier for the message
 * @property {'user'|'assistant'} sender - Identifies the message sender
 * @property {string} content - The text content of the message
 * @property {Date} timestamp - When the message was created/sent
 * @property {string[]} [sources] - Source references for AI-generated responses
 */

/**
 * Creates a new ChatMessage instance
 * @param {Object} params - Parameters for creating a ChatMessage
 * @param {string} params.id - Unique identifier for the message
 * @param {'user'|'assistant'} params.sender - Identifies the message sender
 * @param {string} params.content - The text content of the message
 * @param {Date} [params.timestamp] - When the message was created/sent (defaults to now)
 * @param {string[]} [params.sources] - Source references for AI-generated responses
 * @returns {ChatMessage} A new ChatMessage instance
 */
export const createChatMessage = ({ id, sender, content, timestamp = new Date(), sources = [] }) => {
  return {
    id,
    sender,
    content,
    timestamp: timestamp instanceof Date ? timestamp : new Date(timestamp),
    sources
  };
};

/**
 * Validates if an object conforms to the ChatMessage structure
 * @param {any} obj - Object to validate
 * @returns {boolean} True if the object is a valid ChatMessage
 */
export const isValidChatMessage = (obj) => {
  return (
    typeof obj === 'object' &&
    obj !== null &&
    typeof obj.id === 'string' &&
    (obj.sender === 'user' || obj.sender === 'assistant') &&
    typeof obj.content === 'string' &&
    obj.timestamp instanceof Date &&
    Array.isArray(obj.sources)
  );
};

export default { createChatMessage, isValidChatMessage };