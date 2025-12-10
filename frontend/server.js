// This is a workaround for Docusaurus API proxying
// The docusaurus.config.js doesn't directly support proxy configuration
// Instead, we can create a custom server that handles the proxying

const path = require('path');

module.exports = {
  // The rest of your docusaurus.config.js content stays the same
  // This webpack config will be merged with the existing one
  webpack: {
    devServer: {
      proxy: {
        '/api': {
          target: 'http://localhost:8000',
          changeOrigin: true,
          secure: false,
        },
      },
    },
  },
};