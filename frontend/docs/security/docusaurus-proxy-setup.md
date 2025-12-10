# Docusaurus Development Proxy Configuration

This guide explains how to configure the development proxy for API requests in the Docusaurus documentation site.

## Why Use a Proxy?

When developing locally, you need a way to make API requests from the frontend (running on http://localhost:3000) to your backend (likely running on http://localhost:8000). Browsers block cross-origin requests (CORS) by default, so you need a proxy to forward requests from the Docusaurus development server to your backend.

## Setting Up the Proxy

To enable API request proxying in development, add the following configuration to your `docusaurus.config.js` file:

```javascript
// docusaurus.config.js
const config = {
  // ... other config options
  themes: [
    // ... other themes
  ],
  plugins: [
    // ... other plugins
  ],
  themeConfig: {
    // ... other theme config
  },
  
  // Add this section for development proxy
  webpack: {
    setupMiddlewares: (middlewares, options) => {
      // Only add proxy in development mode
      if (process.env.NODE_ENV === 'development') {
        middlewares.unshift(
          require('http-proxy-middleware').createProxyMiddleware({
            pathFilter: ['/api'],  // Proxy all requests starting with /api
            target: process.env.REACT_APP_API_BASE_URL || 'http://localhost:8000',
            changeOrigin: true,
            logLevel: 'debug',  // Set to 'debug' to see proxy logs during development
            onProxyReq: (proxyReq, req, res) => {
              console.log('Proxying request:', req.method, req.url);
            },
            onProxyRes: (proxyRes, req, res) => {
              console.log('Received response:', proxyRes.statusCode, req.url);
            },
          })
        );
      }
      return middlewares;
    }
  }
};
```

## Required Dependencies

First, you need to install the http-proxy-middleware package:

```bash
npm install --save-dev http-proxy-middleware
```

## Environment Variables

Make sure your `.env` file contains the appropriate API base URL:

```bash
# .env
REACT_APP_API_BASE_URL=http://localhost:8000
```

## Making API Requests

With this configuration in place, you can make API requests from your frontend components to `/api/...` endpoints without CORS issues during development. The proxy will forward these requests to your backend service.

For example, in your components:

```javascript
import config from './services/config';

const apiUrl = config.getApiBaseUrl();
const response = await fetch(`${apiUrl}/api/chat/query`, {
  method: 'POST',
  headers: {
    'Content-Type': 'application/json',
  },
  body: JSON.stringify({
    query: currentInput,
    selected_text: selectedText,
    chapter_id: chapterId,
  })
});
```

The proxy configuration will forward these requests appropriately during development, while production builds will use the actual production API endpoint.

## Production Considerations

In production, you won't need the proxy as your Docusaurus site and backend will likely be served from the same domain or proper CORS headers will be configured.