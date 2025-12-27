/**
 * API service to connect with rag-backend
 */

import { API_ENDPOINTS } from '../config/constants';

const apiService = {
  /**
   * Send a query to the rag-backend
   */
  async sendQuery(query, selectedText = null, context = {}, retryCount = 0) {
    // Input validation
    if (!query || typeof query !== 'string' || query.trim().length === 0) {
      throw new Error('Query is required and must be a non-empty string');
    }

    if (query.length > 10000) {
      throw new Error('Query is too long. Please limit to 10,000 characters.');
    }

    if (selectedText && (typeof selectedText !== 'string' || selectedText.length > 1000)) {
      throw new Error('Selected text must be a string with max 1000 characters');
    }

    const startTime = Date.now();

    // Timeout promise
    const timeoutPromise = new Promise((_, reject) => {
      setTimeout(() => {
        reject(new Error('Request timeout after 30 seconds'));
      }, 30000);
    });

    try {
      // ✅ FIXED: Prepare request body
      const requestBody = {
        query: query.trim(),
        selected_text: selectedText ? selectedText.trim() : null,
      };

      console.log('Sending request to:', API_ENDPOINTS.CHAT);
      console.log('Request body:', requestBody);

      // Make the fetch request
      const response = await Promise.race([
        fetch(API_ENDPOINTS.CHAT, {
          method: 'POST',
          headers: {
            'Content-Type': 'application/json',
          },
          body: JSON.stringify(requestBody),
        }),
        timeoutPromise
      ]);

      console.log('Response status:', response.status);

      if (!response.ok) {
        const errorText = await response.text();
        console.error('Error response:', errorText);
        throw new Error(`HTTP error! status: ${response.status}, message: ${errorText}`);
      }

      const data = await response.json();
      console.log('Response data:', data);

      const responseTime = Date.now() - startTime;
      console.log(`API Response Time: ${responseTime}ms`);

      // ✅ FIXED: Better validation
      if (!data || typeof data !== 'object') {
        throw new Error('Invalid response format from server');
      }

      // ✅ CRITICAL FIX: Check both answer and sources
      if (typeof data.answer !== 'string') {
        console.error('Invalid response structure:', data);
        throw new Error('Backend response missing "answer" field');
      }

      // ✅ Ensure sources is always an array
      if (!Array.isArray(data.sources)) {
        data.sources = [];
      }

      return {
        answer: data.answer,
        sources: data.sources,
        timestamp: new Date().toISOString()
      };

    } catch (error) {
      const responseTime = Date.now() - startTime;
      console.error(`API Error after ${responseTime}ms:`, error);

      // Retry logic for network errors
      if (retryCount < 2 && 
          (error.message.includes('network') || 
           error.message.includes('fetch') || 
           error.message.includes('timeout'))) {
        console.log(`Retrying request (${retryCount + 1}/2)...`);
        await new Promise(resolve => setTimeout(resolve, 1000 * (retryCount + 1)));
        return this.sendQuery(query, selectedText, context, retryCount + 1);
      }

      // Re-throw for UI to handle
      throw error;
    }
  },

  /**
   * Check backend health
   */
  async checkHealth() {
    try {
      const response = await fetch(API_ENDPOINTS.HEALTH || '/health', {
        method: 'GET',
      });

      if (!response.ok) {
        throw new Error(`Health check failed: ${response.status}`);
      }

      return await response.json();
    } catch (error) {
      console.error('Health check error:', error);
      throw error;
    }
  }
};

export default apiService;