import React, { useState } from 'react';
import axios from 'axios';
import { useAuth } from '../contexts/AuthContext';
import ExecutionEnvironment from '@docusaurus/ExecutionEnvironment';

const ChapterControls = ({ chapterId, onContentUpdate }) => {
  const [isPersonalizing, setIsPersonalizing] = useState(false);
  const [isTranslating, setIsTranslating] = useState(false);
  const [showUrduModal, setShowUrduModal] = useState(false);
  const [urduContent, setUrduContent] = useState('');
  const { user, isAuthenticated } = useAuth();

  // Get the user ID from the auth context
  const userId = user?.user_id;

  const handlePersonalize = async () => {
    if (!isAuthenticated || !userId) {
      alert('Please log in to use personalization features');
      return;
    }

    if (!ExecutionEnvironment.canUseDOM) {
      alert('Personalization is only available in the browser');
      return;
    }

    setIsPersonalizing(true);
    try {
      const token = localStorage.getItem('auth_token');
      const response = await axios.post(
        '/api/personalize/chapter',
        {
          chapter_id: chapterId,
          user_id: userId
        },
        {
          headers: {
            'Authorization': `Bearer ${token}`,
            'Content-Type': 'application/json'
          }
        }
      );

      if (onContentUpdate) {
        onContentUpdate(response.data.content);
      }

      // Show success message
      alert('Content has been personalized based on your profile!');
    } catch (error) {
      console.error('Personalization error:', error);
      alert('Error personalizing content. Please try again. Details: ' + (error.response?.data?.detail || error.message));
    } finally {
      setIsPersonalizing(false);
    }
  };

  const handleTranslateToUrdu = async () => {
    if (!isAuthenticated) {
      alert('Please log in to use translation features');
      return;
    }

    if (!ExecutionEnvironment.canUseDOM) {
      alert('Translation is only available in the browser');
      return;
    }

    setIsTranslating(true);
    try {
      const token = localStorage.getItem('auth_token');
      const response = await axios.post(
        '/api/translate/chapter-urdu',
        {
          chapter_id: chapterId
        },
        {
          headers: {
            'Authorization': `Bearer ${token}`,
            'Content-Type': 'application/json'
          }
        }
      );

      setUrduContent(response.data.urdu_content);
      setShowUrduModal(true);
    } catch (error) {
      console.error('Translation error:', error);
      alert('Error translating content. Please try again. Details: ' + (error.response?.data?.detail || error.message));
    } finally {
      setIsTranslating(false);
    }
  };

  const closeModal = () => {
    setShowUrduModal(false);
    setUrduContent('');
  };

  return (
    <div className="chapter-controls margin-bottom--lg">
      {isAuthenticated && (
        <div className="button-group">
          <button
            className="button button--primary"
            onClick={handlePersonalize}
            disabled={isPersonalizing}
            title="Personalize content based on your profile and technical background"
          >
            {isPersonalizing ? (
              <>
                <span className="margin-right--sm">🔄</span>
                Personalizing...
              </>
            ) : (
              'Personalize Content'
            )}
          </button>

          <button
            className="button button--success"
            onClick={handleTranslateToUrdu}
            disabled={isTranslating}
            title="Translate content to Urdu"
          >
            {isTranslating ? (
              <>
                <span className="margin-right--sm">🔄</span>
                Translating...
              </>
            ) : (
              'اردو میں ترجمہ'
            )}
          </button>
        </div>
      )}

      {/* Urdu Translation Modal */}
      {showUrduModal && (
        <div className="modal-overlay" onClick={closeModal}>
          <div className="modal-content" onClick={(e) => e.stopPropagation()}>
            <div className="modal-header">
              <h5>اردو ترجمہ</h5>
              <button className="close-button" onClick={closeModal}>×</button>
            </div>
            <div className="modal-body">
              <div className="urdu-content" dir="rtl">
                {urduContent}
              </div>
            </div>
            <div className="modal-footer">
              <button className="button button--secondary" onClick={closeModal}>Close</button>
            </div>
          </div>
        </div>
      )}
    </div>
  );
};

export default ChapterControls;