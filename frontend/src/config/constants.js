/**
 * Constants for API endpoints and UI settings
 */

// API endpoints for rag-backend integration
export const API_ENDPOINTS = {
  CHAT: '/api/chat',  // Endpoint for chat queries to rag-backend
  HEALTH: '/health'   // Endpoint for health checks
};

// UI configuration constants
export const UI_CONSTANTS = {
  // Chat window settings
  CHAT_WINDOW_WIDTH: 400,      // Width of the chat window in pixels
  CHAT_WINDOW_HEIGHT: 500,     // Height of the chat window in pixels
  FLOATING_BUTTON_SIZE: 60,    // Size of the floating chat button in pixels
  
  // Animation settings
  ANIMATION_DURATION: 300,     // Duration for UI animations in milliseconds
  
  // Responsive breakpoints
  BREAKPOINTS: {
    MOBILE: 768,    // Max width for mobile devices
    TABLET: 1024,   // Max width for tablet devices
    DESKTOP: 1200   // Min width for desktop devices
  },
  
  // Text selection
  MAX_SELECTED_TEXT_LENGTH: 1000  // Maximum length of selected text to send to backend
};

// Error messages
export const ERROR_MESSAGES = {
  NETWORK_ERROR: 'Network error occurred. Please check your connection.',
  TIMEOUT_ERROR: 'Request timed out. Please try again.',
  INVALID_RESPONSE: 'Invalid response from server.',
  CHAT_UNAVAILABLE: 'Chat service is temporarily unavailable.'
};

// Default configuration values
export const DEFAULT_CONFIG = {
  THEME: 'light',
  FONT_SIZE: 'medium',
  SPACING: 'normal'
};