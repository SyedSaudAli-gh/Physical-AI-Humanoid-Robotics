// Test script to verify configuration is working correctly
// This is just a demonstration of how you would test your configuration

import config from './services/config';

console.log('Environment Configuration Verification');
console.log('===================================');

console.log('NODE_ENV:', process.env.NODE_ENV);
console.log('IS_PRODUCTION:', config.IS_PRODUCTION);
console.log('API_BASE_URL:', config.API_BASE_URL);
console.log('PROD_API_BASE_URL:', config.PROD_API_BASE_URL);
console.log('Resolved API Base URL:', config.getApiBaseUrl());
console.log('Google Analytics ID:', config.GOOGLE_ANALYTICS_ID);
console.log('Enable Debug Logs:', config.ENABLE_DEBUG_LOGS);

// Verify API functions are working
import { getApiBaseUrl, fetchModules, fetchModuleChapters, fetchChapterContent } from './services/api';

console.log('\nAPI Configuration:');
console.log('API Base URL from api.js:', getApiBaseUrl());

// Note: Actual API calls would require a running backend
// This is just to verify the functions are properly exported
console.log('API functions available:', {
  fetchModules: typeof fetchModules === 'function',
  fetchModuleChapters: typeof fetchModuleChapters === 'function',
  fetchChapterContent: typeof fetchChapterContent === 'function'
});

console.log('\nConfiguration verification complete!');
console.log('Remember: This is a frontend application.');
console.log('Environment variables are resolved at build time in Docusaurus.');
console.log('For actual API functionality, you need a running backend service.');