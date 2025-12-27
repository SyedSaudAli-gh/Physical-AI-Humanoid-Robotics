/**
 * API service to connect with rag-backend
 * Provides methods to interact with the backend API endpoints
 */

import { API_ENDPOINTS } from '../config/constants';

/**
 * Service for API communication with the rag-backend
 */
const apiService = {
  /**
   * Send a query to the rag-backend and receive an AI response with source references
   * @param {string} query - The user's query
   * @param {string} [selectedText] - Optional selected text to provide context
   * @param {Object} [context] - Additional context like module/chapter info
   * @param {number} [retryCount=0] - Number of retries attempted
   * @returns {Promise<Object>} The AI response with sources and metadata
   */
  async sendQuery(query, selectedText = null, context = {}, retryCount = 0) {
    // Input validation
    if (!query || typeof query !== 'string' || query.trim().length === 0) {
      throw new Error('Query is required and must be a non-empty string');
    }

    // Add validation for extremely long queries (edge case)
    if (query.length > 10000) {
      throw new Error('Query is too long. Please limit your query to 10,000 characters.');
    }

    if (selectedText && (typeof selectedText !== 'string' || selectedText.length > 1000)) {
      throw new Error('Selected text must be a string with maximum length of 1000 characters');
    }

    // Track start time for performance metrics
    const startTime = Date.now();

    // Create a timeout promise to prevent hanging requests
    const timeoutPromise = new Promise((_, reject) => {
      setTimeout(() => {
        reject(new Error('Request timeout after 30 seconds'));
      }, 30000); // 30 seconds timeout
    });

    try {
      // Prepare the request body with query and context
      const requestBody = {
        query: query.trim(),
        selected_text: selectedText ? selectedText.trim() : null,
        context
      };

      // Race the fetch request against the timeout
      const response = await Promise.race([
        fetch(API_ENDPOINTS.CHAT, {
          method: 'POST',
          headers: {
            'Content-Type': 'application/json',
            // Add any authentication headers if needed
            // 'Authorization': `Bearer ${authToken}`,
          },
          body: JSON.stringify(requestBody),
        }),
        timeoutPromise
      ]);

      if (!response.ok) {
        const errorData = await response.json().catch(() => ({}));
        throw new Error(`HTTP error! status: ${response.status}, message: ${errorData.error || 'Unknown error'}`);
      }

      const data = await response.json();

      // Calculate response time
      const responseTime = Date.now() - startTime;

      // Log performance metrics
      console.log(`API Response Time: ${responseTime}ms for query: ${query.substring(0, 50)}...`);

      // Check if response time exceeds 5 seconds and log a warning
      if (responseTime > 5000) {
        console.warn(`Slow response detected: ${responseTime}ms for query: ${query.substring(0, 50)}...`);
      }

      // Validate response structure
      if (!data || typeof data !== 'object') {
        throw new Error('Invalid response format from server');
      }

      if (!data.answer) {
        console.warn('Response from backend did not include an answer:', data);
      }

      return data;
    } catch (error) {
      const responseTime = Date.now() - startTime;
      console.error(`API Error after ${responseTime}ms:`, error);

      // If it's a timeout error, throw a specific message
      if (error.message.includes('timeout')) {
        throw new Error('Request timed out. Please check your connection to the backend and try again.');
      }
      // If it's our validation error, re-throw it
      if (error.message.includes('Query is required') || error.message.includes('Selected text must be a string')) {
        throw error;
      }

      // Implement retry mechanism for network errors (up to 3 attempts)
      if (retryCount < 2 &&
          (error.message.includes('network') ||
           error.message.includes('fetch') ||
           error.message.includes('connect') ||
           error.message.includes('AI service'))) {
        console.log(`Retrying request (${retryCount + 1}/2)...`);
        await new Promise(resolve => setTimeout(resolve, 1000 * (retryCount + 1))); // Exponential backoff
        return this.sendQuery(query, selectedText, context, retryCount + 1);
      }

      // For network errors or other issues, provide a user-friendly message
      throw new Error('Unable to connect to the AI service. Please check your connection and try again.');
    }
  },

  /**
   * Check the health/status of the rag-backend service
   * @returns {Promise<Object>} Health check response
   */
  async checkHealth() {
    try {
      const response = await fetch(API_ENDPOINTS.HEALTH, {
        method: 'GET',
      });

      if (!response.ok) {
        throw new Error(`Health check failed with status: ${response.status}`);
      }

      return await response.json();
    } catch (error) {
      console.error('Health check error:', error);
      throw error;
    }
  }
};

export default apiService;