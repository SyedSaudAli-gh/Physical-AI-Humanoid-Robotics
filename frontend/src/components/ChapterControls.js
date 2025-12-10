import React, { useState } from 'react';
import axios from 'axios';
import { useAuth } from '../contexts/AuthContext';

const ChapterControls = ({ chapterId, onContentUpdate }) => {
  const [isPersonalizing, setIsPersonalizing] = useState(false);
  const [isTranslating, setIsTranslating] = useState(false);
  const [showUrduModal, setShowUrduModal] = useState(false);
  const [urduContent, setUrduContent] = useState('');
  const { isAuthenticated } = useAuth();

  const handlePersonalize = async () => {
    if (!isAuthenticated) {
      alert('Please log in to use personalization features');
      return;
    }

    setIsPersonalizing(true);
    try {
      const token = localStorage.getItem('auth_token');
      const response = await axios.post(
        '/api/personalize/chapter',
        {
          chapter_id: chapterId,
          user_id: localStorage.getItem('user_id')  // This would need to be set when user logs in
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
    } catch (error) {
      console.error('Personalization error:', error);
      alert('Error personalizing content. Please try again.');
    } finally {
      setIsPersonalizing(false);
    }
  };

  const handleTranslateToUrdu = async () => {
    if (!isAuthenticated) {
      alert('Please log in to use translation features');
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
      alert('Error translating content. Please try again.');
    } finally {
      setIsTranslating(false);
    }
  };

  const closeModal = () => {
    setShowUrduModal(false);
    setUrduContent('');
  };

  return (
    <div className="chapter-controls">
      {isAuthenticated && (
        <>
          <button
            className="btn btn-primary btn-sm me-2"
            onClick={handlePersonalize}
            disabled={isPersonalizing}
          >
            {isPersonalizing ? 'Personalizing...' : 'Personalize Content'}
          </button>

          <button
            className="btn btn-success btn-sm"
            onClick={handleTranslateToUrdu}
            disabled={isTranslating}
          >
            {isTranslating ? 'Translating...' : 'اردو میں ترجمہ'}
          </button>
        </>
      )}

      {/* Urdu Translation Modal */}
      {showUrduModal && (
        <div className="modal-overlay" onClick={closeModal}>
          <div className="modal-content" onClick={(e) => e.stopPropagation()}>
            <div className="modal-header">
              <h5>اردو ترجمہ</h5>
              <button className="close-btn" onClick={closeModal}>×</button>
            </div>
            <div className="modal-body">
              <div className="urdu-content" dir="rtl">
                {urduContent}
              </div>
            </div>
            <div className="modal-footer">
              <button className="btn btn-secondary" onClick={closeModal}>Close</button>
            </div>
          </div>
        </div>
      )}
    </div>
  );
};

// Add styles through a separate CSS file or by adding style tags to the document head
if (typeof document !== 'undefined') {
  const styles = `
    .chapter-controls {
      margin: 20px 0;
      padding: 10px;
      background-color: #f8f9fa;
      border-radius: 5px;
      display: flex;
      gap: 10px;
      flex-wrap: wrap;
    }

    .modal-overlay {
      position: fixed;
      top: 0;
      left: 0;
      right: 0;
      bottom: 0;
      background-color: rgba(0, 0, 0, 0.5);
      display: flex;
      justify-content: center;
      align-items: center;
      z-index: 1000;
    }

    .modal-content {
      background: white;
      border-radius: 8px;
      width: 80%;
      max-width: 800px;
      max-height: 80vh;
      overflow-y: auto;
      position: relative;
    }

    .modal-header {
      padding: 1rem;
      border-bottom: 1px solid #dee2e6;
      display: flex;
      justify-content: space-between;
      align-items: center;
    }

    .close-btn {
      background: none;
      border: none;
      font-size: 1.5rem;
      cursor: pointer;
    }

    .modal-body {
      padding: 1rem;
    }

    .urdu-content {
      font-family: 'Noto Nastaliq Urdu', 'Jameel Noori Nastaleeq', 'Urdu Typesetting', serif;
      font-size: 1.1rem;
      line-height: 1.8;
    }

    .modal-footer {
      padding: 1rem;
      border-top: 1px solid #dee2e6;
      text-align: right;
    }
  `;

  const styleSheet = document.createElement("style");
  styleSheet.type = "text/css";
  styleSheet.innerText = styles;
  document.head.appendChild(styleSheet);
}

export default ChapterControls;