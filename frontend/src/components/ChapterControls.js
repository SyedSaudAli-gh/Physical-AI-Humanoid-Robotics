import React, { useState } from 'react';
import { useUser } from '../contexts/UserContext';
import ChapterDifficultySelector from './ChapterDifficultySelector';
import UrduTranslateButton from './UrduTranslateButton';

// Controls for chapter pages including translation functionality
const ChapterControls = ({
  chapterId,
  currentContent,
  onDifficultyChange
}) => {
  const { user, preferences, loading } = useUser();
  const [showTranslationControls, setShowTranslationControls] = useState(false);

  return (
    <div className="chapter-controls margin-bottom--lg">
      <div className="row">
        <div className="col col--6">
          <ChapterDifficultySelector 
            chapterId={chapterId} 
            onDifficultyChange={onDifficultyChange} 
          />
        </div>
        
        <div className="col col--6">
          <div className="button-group">
            <UrduTranslateButton
              content={currentContent}
              isLoggedIn={!!user}
            />

            <button
              className="button button--outline"
              onClick={() => setShowTranslationControls(!showTranslationControls)}
            >
              More Options
            </button>
          </div>
        </div>
      </div>
      
      {showTranslationControls && (
        <div className="alert alert--info margin-top--sm">
          <p><strong>Translation Info:</strong></p>
          <ul>
            <li>Translations are generated using Qwen AI technology</li>
            <li>Translation accuracy may vary for technical terminology</li>
            <li>For best results, use the original English content for complex topics</li>
          </ul>
        </div>
      )}
    </div>
  );
};

export default ChapterControls;