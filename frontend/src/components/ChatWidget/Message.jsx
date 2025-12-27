import React from 'react';
import '../ChatWidget/styles.css';

const Message = ({ text, isUser, sources = [] }) => {
  return (
    <div
      className={`message ${isUser ? 'user' : 'bot'}`}
      role="logitem"
      aria-label={isUser ? "User message" : "Assistant response"}
    >
      <div>{text}</div>
      {sources && sources.length > 0 && (
        <div className="source-references" role="region" aria-label="Source references">
          <h4>Sources:</h4>
          {sources.map((source, index) => (
            <a
              key={index}
              href={source.url}
              className="source-link"
              target="_blank"
              rel="noopener noreferrer"
              aria-label={`Source: ${source.title}`}
            >
              {source.title}
            </a>
          ))}
        </div>
      )}
    </div>
  );
};

export default Message;