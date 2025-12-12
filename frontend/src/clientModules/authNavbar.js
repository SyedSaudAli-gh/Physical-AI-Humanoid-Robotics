import React from 'react';
import { createRoot } from 'react-dom/client';
import AuthNavbarItem from '../components/AuthNavbarItem';

// Function to render the AuthNavbarItem in the placeholder
function renderAuthNavbar() {
  const placeholder = document.getElementById('auth-navbar-placeholder');
  if (placeholder) {
    // Create a container div to render the component
    const container = document.createElement('div');
    placeholder.appendChild(container);

    // Render the component (AuthProvider is handled at root level)
    const root = createRoot(container);
    root.render(<AuthNavbarItem />);
  }
}

// Wait for the DOM to be ready before rendering
if (document.readyState === 'loading') {
  document.addEventListener('DOMContentLoaded', renderAuthNavbar);
} else {
  renderAuthNavbar();
}

// For SPA navigation support
window.addEventListener('load', () => {
  // Re-render when the page content changes
  if (window.docusaurus) {
    window.docusaurus.eventEmitter?.on('routeDidUpdate', renderAuthNavbar);
  }
});