// config.js - Centralized configuration for environment variables
// This file defines all configuration values used throughout the application

// Base API URL configuration
// In a Docusaurus project, we can read environment variables during build time
// and pass them as config values through docusaurus.config.js

const config = {
  // API configuration
  API_BASE_URL: process.env.REACT_APP_API_BASE_URL || 
                process.env.API_BASE_URL || 
                'http://localhost:8000',
                
  PROD_API_BASE_URL: process.env.REACT_APP_PROD_API_BASE_URL ||
                   process.env.PROD_API_BASE_URL ||
                   'https://yourdomain.com/api',
                   
  // Determine if we're in production based on NODE_ENV or build context
  IS_PRODUCTION: process.env.NODE_ENV === 'production',
  
  // Other configuration values can be added here
  // For example, analytics IDs if needed
  GOOGLE_ANALYTICS_ID: process.env.REACT_APP_GA_MEASUREMENT_ID || null,
  
  // Feature flags
  ENABLE_DEBUG_LOGS: process.env.NODE_ENV !== 'production',
  
  // Timeouts and limits
  API_TIMEOUT_MS: 30000, // 30 seconds
  MAX_FILE_SIZE_UPLOAD: 10 * 1024 * 1024, // 10MB in bytes
};

// Helper function to get the appropriate API base URL based on current environment
config.getApiBaseUrl = function() {
  return this.IS_PRODUCTION ? this.PROD_API_BASE_URL : this.API_BASE_URL;
};

export default config;