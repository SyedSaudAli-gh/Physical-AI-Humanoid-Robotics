/**
 * Utility functions for API request formatting and handling
 */

/**
 * Formats a query for the rag-backend API
 * @param {string} query - The user's query
 * @param {string} [selectedText] - Optional selected text for context
 * @param {Object} [context] - Additional context information
 * @returns {Object} Formatted request object
 */
export const formatQueryRequest = (query, selectedText = null, context = {}) => {
  return {
    query: query.trim(),
    selected_text: selectedText ? selectedText.trim() : null,
    context: {
      ...context
    }
  };
};

/**
 * Formats the response from the rag-backend API
 * @param {Object} response - Raw response from the backend
 * @returns {Object} Formatted response object
 */
export const formatApiResponse = (response) => {
  return {
    answer: response.answer || '',
    sources: response.sources || [],
    timestamp: response.timestamp || new Date().toISOString()
  };
};

/**
 * Validates the query request before sending to backend
 * @param {Object} request - Request object to validate
 * @returns {Array<string>} Array of validation errors, empty if valid
 */
export const validateQueryRequest = (request) => {
  const errors = [];

  if (!request.query || typeof request.query !== 'string' || request.query.trim().length === 0) {
    errors.push('Query is required and must be a non-empty string');
  }

  if (request.selected_text && typeof request.selected_text !== 'string') {
    errors.push('Selected text must be a string if provided');
  }

  if (request.context && typeof request.context !== 'object') {
    errors.push('Context must be an object if provided');
  }

  return errors;
};

/**
 * Sanitizes user input to prevent XSS and other injection attacks
 * @param {string} input - Input string to sanitize
 * @returns {string} Sanitized input string
 */
export const sanitizeInput = (input) => {
  if (typeof input !== 'string') {
    return '';
  }

  // Basic sanitization to prevent script tags and other potentially harmful content
  return input
    .replace(/<script\b[^<]*(?:(?!<\/script>)<[^<]*)*<\/script>/gi, '')  // Remove script tags
    .replace(/<iframe\b[^<]*(?:(?!<\/iframe>)<[^<]*)*<\/iframe>/gi, '')  // Remove iframe tags
    .replace(/javascript:/gi, '')  // Remove javascript: protocol
    .replace(/vbscript:/gi, '')    // Remove vbscript: protocol
    .replace(/on\w+="[^"]*"/gi, '') // Remove event handlers like onclick, onload, etc.
    .trim();
};

/**
 * Prepares headers for API requests
 * @returns {Object} Headers object
 */
export const getApiHeaders = () => {
  return {
    'Content-Type': 'application/json',
    // Add authorization header if available
    ...(localStorage.getItem('authToken') && {
      'Authorization': `Bearer ${localStorage.getItem('authToken')}`
    })
  };
};

/**
 * Creates an API request with timeout
 * @param {string} url - API endpoint URL
 * @param {Object} options - Request options (method, headers, body, etc.)
 * @param {number} [timeoutMs=30000] - Timeout in milliseconds (default 30s)
 * @returns {Promise<Response>} API response
 */
export const fetchWithTimeout = async (url, options = {}, timeoutMs = 30000) => {
  // Create a timeout promise
  const timeoutPromise = new Promise((_, reject) => {
    setTimeout(() => {
      reject(new Error(`Request timeout after ${timeoutMs}ms`));
    }, timeoutMs);
  });

  // Race the fetch request against the timeout
  const response = await Promise.race([
    fetch(url, options),
    timeoutPromise
  ]);

  return response;
};