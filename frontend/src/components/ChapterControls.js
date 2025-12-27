import React, { useState } from 'react';
import axios from 'axios';
import { useAuth } from '../contexts/AuthContext';
import ExecutionEnvironment from '@docusaurus/ExecutionEnvironment';

const ChapterControls = ({ chapterId, onContentUpdate }) => {
  const [isPersonalizing, setIsPersonalizing] = useState(false);
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
        </div>
      )}
    </div>
  );
};

export default ChapterControls;