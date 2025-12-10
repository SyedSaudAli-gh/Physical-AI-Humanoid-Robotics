# API Security and Communication Guide

This document outlines how to properly handle API communication between the frontend and backend while maintaining security.

## Frontend to Backend Communication

For security reasons, all external API calls (OpenAI, Cohere, Gemini, etc.) should be proxied through your backend service rather than called directly from the frontend. This prevents exposing your API keys to the client-side.

## API Proxy Architecture

### Backend API Endpoints

Your backend should expose the following proxy endpoints:

```
POST /api/openai/chat
POST /api/cohere/generate
POST /api/gemini/process
POST /api/qdrant/search
```

### Frontend Implementation

From the frontend, you'll make requests to your own backend which will then make requests to external services:

```javascript
// Example of how to call the backend proxy
const response = await fetch('/api/openai/chat', {
  method: 'POST',
  headers: {
    'Content-Type': 'application/json',
    // Include authentication headers if implemented
  },
  body: JSON.stringify({
    messages: [
      { role: 'user', content: 'Hello, how are you?' }
    ]
  })
});

const data = await response.json();
```

## Frontend Environment Variables

For frontend-specific configuration, create a `.env` file in your frontend directory with settings like:

```bash
# Base URL for your backend API
REACT_APP_API_BASE_URL=http://localhost:8000
REACT_APP_PROD_API_BASE_URL=https://yourdomain.com/api
```

> Note: In a Docusaurus project like this one, the environment variables work differently than in a Create React App project. For Docusaurus, you may need to use different methods to configure environment-specific settings.

## Docusaurus Environment Configuration

Since this is a Docusaurus project, you can pass environment-specific variables in several ways:

### Method 1: Using Site Config
Add environment-specific values in `docusaurus.config.js`:

```javascript
const config = {
  // ... other config
  customFields: {
    // Custom fields that you can use in your components
    API_BASE_URL: process.env.API_BASE_URL || 'http://localhost:8000',
  },
};
```

### Method 2: Using Plugins
Use a Docusaurus plugin to inject environment variables at build time.

## Security Considerations

- Never put API keys in frontend code
- Always validate and sanitize input on the backend
- Implement proper rate limiting on API endpoints
- Use HTTPS in production for all API communications
- Implement proper authentication and authorization if needed