# Environment Variables Security Guide

This document outlines best practices for handling environment variables and API keys securely in our Physical AI & Humanoid Robotics project.

## Backend Environment Variables

### Required Variables

```bash
# OpenAI API Key
OPENAI_API_KEY=your_openai_api_key_here

# Cohere API Key
COHERE_API_KEY=your_cohere_api_key_here

# Qdrant Vector Database
QDRANT_URL=your_qdrant_cluster_url
QDRANT_API_KEY=your_qdrant_api_key

# Database Connection
DATABASE_URL=your_database_connection_string

# Authentication Secret
BETTER_AUTH_SECRET=your_auth_secret

# Google Gemini API Key
GEMINI_API_KEY=your_gemini_api_key
```

### Security Best Practices

1. **Never commit API keys to version control**
   - Use `.env` files that are ignored by git
   - Add `.env` and similar patterns to your `.gitignore` file

2. **Use environment-specific configurations**
   - Use different API keys for development, staging, and production
   - Implement environment variable validation

3. **Limit permissions**
   - Use API keys with minimal required permissions
   - Rotate keys regularly

4. **Secure storage**
   - Use secure vaults (like HashiCorp Vault, AWS Secrets Manager, etc.) for production
   - Never log API keys or sensitive information

### Recommended .gitignore entries

Add these patterns to your backend's `.gitignore` file:

```
.env
.env.local
.env.*.local
config/env.local
secrets/
*.key
```

### Backend Implementation

When implementing your backend, use a configuration loader that securely handles environment variables:

```javascript
// Example configuration loading
require('dotenv').config();

const config = {
  openai: {
    apiKey: process.env.OPENAI_API_KEY
  },
  cohere: {
    apiKey: process.env.COHERE_API_KEY
  },
  qdrant: {
    url: process.env.QDRANT_URL,
    apiKey: process.env.QDRANT_API_KEY
  },
  database: {
    url: process.env.DATABASE_URL
  },
  auth: {
    secret: process.env.BETTER_AUTH_SECRET
  },
  gemini: {
    apiKey: process.env.GEMINI_API_KEY
  }
};
```

## Frontend Considerations

> ⚠️ Important: Never expose backend API keys to the frontend code. All API calls to external services (OpenAI, Cohere, etc.) should be proxied through your backend.

### Recommended Architecture

1. Frontend makes requests to your backend API
2. Backend handles all external API calls
3. Backend returns processed data to frontend

This prevents exposing your API keys to clients and potential misuse.

### Environment Variables in Frontend

For frontend-specific environment variables (like API URLs), create a separate `.env` file:

```bash
# Frontend environment variables
REACT_APP_API_BASE_URL=http://localhost:3000/api
REACT_APP_GA_MEASUREMENT_ID=G-XXXXXXXXXX  # Google Analytics ID if needed
```

## Incident Response

If API keys have been exposed:

1. Immediately revoke/regenerate the exposed keys
2. Audit logs for unusual activity
3. Update all references to the new keys
4. Consider security implications and communicate as needed
```