import React from 'react';
import './ErrorMessage.module.css';

const ErrorMessage = ({ message, onRetry, showRetry = true }) => {
  return (
    <div className="error-message-container">
      <div className="error-message-content">
        <div className="error-icon">⚠️</div>
        <div className="error-text">{message}</div>
        {showRetry && onRetry && (
          <button className="retry-button" onClick={onRetry}>
            Try Again
          </button>
        )}
      </div>
    </div>
  );
};

export default ErrorMessage;