import React, { useState, useEffect } from 'react';
import { useAuth } from '../contexts/AuthContext';
import axios from 'axios';

// Urdu translation button component with toggle functionality
const UrduTranslateButton = ({
  content,
  chapterId,
  onTranslationComplete,
  isLoggedIn = true
}) => {
  const [isTranslating, setIsTranslating] = useState(false);
  const [translatedContent, setTranslatedContent] = useState(null);
  const [isUrduView, setIsUrduView] = useState(false);
  const [error, setError] = useState(null);
  const { user, loading: authLoading } = useAuth();

  // Only show the button if the user is logged in
  if (!isLoggedIn || !user) {
    return null;
  }

  const handleTranslate = async () => {
    if ((!content && !chapterId) || isTranslating || authLoading) return;

    setIsTranslating(true);
    setError(null);

    try {
      // If we have a chapterId, use the chapter-specific translation endpoint
      let response;
      if (chapterId) {
        response = await axios.post('/api/translate/chapter-urdu', {
          chapter_id: chapterId
        });
      } else {
        // Otherwise, translate the provided content
        response = await axios.post('/api/translate', {
          content: content,
          target_language: 'ur'
        });
      }

      if (response.data && response.data.urdu_content) {
        setTranslatedContent(response.data.urdu_content);
        setIsUrduView(true);
        if (onTranslationComplete) {
          onTranslationComplete(response.data.urdu_content);
        }
      } else if (response.data && response.data.translated_content) {
        setTranslatedContent(response.data.translated_content);
        setIsUrduView(true);
        if (onTranslationComplete) {
          onTranslationComplete(response.data.translated_content);
        }
      }
    } catch (error) {
      console.error('Translation error:', error);
      setError('ترجمہ میں مسئلہ، دوبارہ کوشش کریں');
      alert('Translation failed: ' + (error.response?.data?.detail || error.message));
    } finally {
      setIsTranslating(false);
    }
  };

  const toggleView = () => {
    setIsUrduView(!isUrduView);
  };

  const toggleTranslation = () => {
    if (isUrduView) {
      // Switch back to English
      setIsUrduView(false);
    } else {
      // Switch to Urdu if we have translation, otherwise fetch it
      if (translatedContent) {
        setIsUrduView(true);
      } else {
        handleTranslate();
      }
    }
  };

  // Handle Escape key to switch back to English
  useEffect(() => {
    const handleEscKey = (e) => {
      if (e.key === 'Escape' && isUrduView) {
        setIsUrduView(false);
      }
    };

    window.addEventListener('keydown', handleEscKey);
    return () => window.removeEventListener('keydown', handleEscKey);
  }, [isUrduView]);

  return (
    <div className="urdu-translation-container">
      <button
        className={`button ${isUrduView ? 'button--secondary' : 'button--primary'}`}
        onClick={toggleTranslation}
        disabled={isTranslating || authLoading}
        title={isUrduView ? 'Switch to English' : 'Translate to Urdu'}
      >
        {isTranslating ? (
          <>
            <span className="margin-right--sm">🔄</span>
            Translating...
          </>
        ) : isUrduView ? (
          'English'
        ) : (
          'اردو'
        )}
      </button>

      {error && (
        <div className="alert alert--danger margin-top--sm" role="alert">
          {error}
        </div>
      )}
    </div>
  );
};

export default UrduTranslateButton;