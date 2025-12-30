/**
 * Constants for API endpoints and UI settings
 */

// ✅ CRITICAL: API endpoints must match backend EXACTLY
export const API_ENDPOINTS = {
  CHAT: 'https://syedsaudali-deploy-rag-backend.hf.space/api/chat',  // ✅ FULL URL
  HEALTH: 'https://syedsaudali-deploy-rag-backend.hf.space/health',
};

// UI configuration constants
export const UI_CONSTANTS = {
  CHAT_WINDOW_WIDTH: 400,
  CHAT_WINDOW_HEIGHT: 500,
  FLOATING_BUTTON_SIZE: 60,
  ANIMATION_DURATION: 300,
  
  BREAKPOINTS: {
    MOBILE: 768,
    TABLET: 1024,
    DESKTOP: 1200
  },
  
  MAX_SELECTED_TEXT_LENGTH: 1000
};

// Error messages
export const ERROR_MESSAGES = {
  NETWORK_ERROR: 'Network error occurred. Please check your connection.',
  TIMEOUT_ERROR: 'Request timed out. Please try again.',
  INVALID_RESPONSE: 'Invalid response from server.',
  CHAT_UNAVAILABLE: 'Chat service is temporarily unavailable.'
};

// Default configuration
export const DEFAULT_CONFIG = {
  THEME: 'light',
  FONT_SIZE: 'medium',
  SPACING: 'normal'
};