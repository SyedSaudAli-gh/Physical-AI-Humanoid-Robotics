# Quickstart Guide: RAG Chatbot - Frontend Integration with Docusaurus

## Prerequisites

- Node.js 18+ installed
- Docusaurus 2+ project set up
- Access to the FastAPI backend running the /api/chat endpoint
- Basic knowledge of React and JavaScript

## Installation

1. Clone or navigate to your Docusaurus project directory:
```bash
cd your-docusaurus-project
```

2. Install required dependencies (if not already present):
```bash
npm install react react-dom
```

## Integration Steps

### 1. Create the ChatWidget Component

Create a new directory and files for the chat component:

```bash
mkdir src/components/ChatWidget
touch src/components/ChatWidget/ChatWidget.jsx
touch src/components/ChatWidget/ChatWindow.jsx
touch src/components/ChatWidget/Message.jsx
touch src/components/ChatWidget/FloatingButton.jsx
touch src/components/ChatWidget/styles.css
```

### 2. Add the Text Selection Hook

Create the hook to capture selected text:

```bash
mkdir src/hooks
touch src/hooks/useTextSelection.js
```

### 3. Add API Service

Create a service to handle API communication:

```bash
mkdir src/services
touch src/services/api.js
```

### 4. Implement the Components

#### ChatWidget.jsx
```jsx
import React, { useState } from 'react';
import FloatingButton from './FloatingButton';
import ChatWindow from './ChatWindow';
import './styles.css';

const ChatWidget = () => {
  const [isOpen, setIsOpen] = useState(false);

  const toggleChat = () => {
    setIsOpen(!isOpen);
  };

  return (
    <div className="chat-widget">
      {isOpen ? (
        <ChatWindow onClose={toggleChat} />
      ) : (
        <FloatingButton onClick={toggleChat} />
      )}
    </div>
  );
};

export default ChatWidget;
```

#### FloatingButton.jsx
```jsx
import React from 'react';

const FloatingButton = ({ onClick }) => {
  return (
    <button className="floating-chat-button" onClick={onClick}>
      <span className="chat-icon">💬</span>
    </button>
  );
};

export default FloatingButton;
```

### 5. Environment Configuration

Add the backend API URL to your environment configuration:

```js
// In your docusaurus.config.js
module.exports = {
  // ... other config
  themeConfig: {
    // ... other theme config
    backendUrl: process.env.BACKEND_URL || 'http://localhost:8000',
  },
};
```

### 6. Register the Component

Add the ChatWidget to your Docusaurus layout by modifying the theme:

```js
// In your src/theme/Layout/index.js or similar
import React from 'react';
import OriginalLayout from '@theme-original/Layout';
import ChatWidget from '@site/src/components/ChatWidget';

export default function Layout(props) {
  return (
    <>
      <OriginalLayout {...props} />
      <ChatWidget />
    </>
  );
}
```

## Running the Application

1. Start your Docusaurus development server:
```bash
npm run start
```

2. Make sure your FastAPI backend is running on the configured endpoint.

3. Visit your Docusaurus site and you should see the floating chat button.

## Testing the Chat Functionality

1. Select some text on any documentation page
2. Click the floating chat button
3. The chat window should appear, and if text was selected, it should be available as context
4. Type a query and submit it
5. You should receive a response from the AI with source references

## Configuration Options

- `BACKEND_URL`: Environment variable to configure the backend API endpoint
- The component supports responsive design for both mobile and desktop
- Loading and error states are handled automatically