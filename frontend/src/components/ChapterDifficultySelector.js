import React, { useState, useEffect } from 'react';
import { useUser } from '../contexts/UserContext';

// Component to allow users to select content difficulty
const ChapterDifficultySelector = ({ chapterId, onDifficultyChange }) => {
  const { preferences, updatePreferences, loading } = useUser();
  const [selectedDifficulty, setSelectedDifficulty] = useState(
    preferences?.chapter_difficulty_override || 'default'
  );

  useEffect(() => {
    // When the preference changes in the context, update the local state
    setSelectedDifficulty(preferences?.chapter_difficulty_override || 'default');
  }, [preferences]);

  const handleDifficultyChange = async (difficulty) => {
    const newDifficulty = difficulty === 'default' ? null : difficulty;
    setSelectedDifficulty(difficulty);
    
    // Update user preferences
    await updatePreferences({ chapter_difficulty_override: newDifficulty });
    
    // Notify parent component of the change
    if (onDifficultyChange) {
      onDifficultyChange(newDifficulty);
    }
  };

  return (
    <div className="difficulty-selector margin-bottom--md">
      <label htmlFor="difficulty-select">Content Difficulty:</label>
      
      <div className="button-group margin-top--sm">
        {[
          { value: 'default', label: 'Auto (Based on Profile)', color: 'outline' },
          { value: 'beginner', label: 'Beginner', color: 'success' },
          { value: 'intermediate', label: 'Intermediate', color: 'secondary' },
          { value: 'advanced', label: 'Advanced', color: 'primary' }
        ].map((option) => (
          <button
            key={option.value}
            className={`button button--${option.color} ${selectedDifficulty === option.value ? 'button--active' : ''}`}
            onClick={() => handleDifficultyChange(option.value)}
            disabled={loading}
            style={{
              margin: '0 0.25rem',
              ...(selectedDifficulty === option.value ? { transform: 'scale(1.05)' } : {})
            }}
          >
            {option.label}
          </button>
        ))}
      </div>
      
      {selectedDifficulty !== 'default' && (
        <div className="alert alert--info margin-top--sm">
          Showing content customized for <strong>{selectedDifficulty}</strong> level learners.
        </div>
      )}
    </div>
  );
};

export default ChapterDifficultySelector;