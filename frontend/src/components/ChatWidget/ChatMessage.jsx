/**
 * ChatMessage Component
 * Displays individual messages in the chat interface
 */

import React from 'react';

const ChatMessage = ({ message, sender }) => {
  const {
    text = '',        // ✅ SAFE DEFAULT
    sources,
    isError,
    timestamp
  } = message || {};

  const formatTime = (date) => {
    if (!date) return '';
    const time = new Date(date);
    return time.toLocaleTimeString([], {
      hour: '2-digit',
      minute: '2-digit',
    });
  };

  const getAriaLabel = () => {
    const safeText = typeof text === 'string' ? text : '';

    if (isError) {
      return `Error message: ${safeText}`;
    }

    const senderText =
      sender === 'user' ? 'Your message' : 'Assistant response';

    return `${senderText}: ${safeText.substring(0, 50)}${
      safeText.length > 50 ? '...' : ''
    }`;
  };

  return (
    <div
      className={`message ${sender} ${isError ? 'error' : ''}`}
      role="listitem"
      aria-label={getAriaLabel()}
    >
      <div className="messageContent">
        {isError ? (
          <div className="errorMessage" role="alert">
            <strong>Error:</strong> {text}
          </div>
        ) : (
          <div className="textContent">{text}</div>
        )}

        {Array.isArray(sources) && sources.length > 0 && !isError && (
          <div className="source-references">
            <details>
              <summary>Sources</summary>
              <ul>
                {sources.map((source, index) => (
                  <li key={index}>
                    {source?.title ? (
                      <a
                        href={source.url || '#'}
                        target="_blank"
                        rel="noopener noreferrer"
                      >
                        {source.title}
                      </a>
                    ) : (
                      <span>{source?.excerpt || 'Source'}</span>
                    )}
                  </li>
                ))}
              </ul>
            </details>
          </div>
        )}
      </div>

      {timestamp && (
        <div className="timestamp">
          {formatTime(timestamp)}
        </div>
      )}
    </div>
  );
};

export default ChatMessage;
