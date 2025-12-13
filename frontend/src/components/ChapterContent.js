import React, { useState, useEffect, useCallback } from 'react';
import { fetchChapterContent } from '../services/api';
import RAGChatbot from './RAGChatbot';
import ChapterControls from './ChapterControls';
import { useUser } from '../contexts/UserContext';
import ExecutionEnvironment from '@docusaurus/ExecutionEnvironment';
import styles from './UrduTranslation.module.css';

// Component for displaying chapter content with RAG chatbot and translation capabilities
const ChapterContent = ({ chapterId }) => {
  const [chapter, setChapter] = useState(null);
  const [currentContent, setCurrentContent] = useState('');
  const [loading, setLoading] = useState(true);
  const [error, setError] = useState(null);
  const [selectedText, setSelectedText] = useState('');
  const { preferences } = useUser();

  // Check if content is currently in Urdu based on URL or user preference
  const isUrduContent = () => {
    if (!ExecutionEnvironment.canUseDOM) {
      return preferences?.preferred_language === 'ur';
    }

    const urlParams = new URLSearchParams(window.location.search);
    return urlParams.get('lang') === 'ur' || preferences?.preferred_language === 'ur';
  };

  const loadChapter = useCallback(async () => {
    try {
      setLoading(true);
      const data = await fetchChapterContent(chapterId, null, isUrduContent() ? 'ur' : null);
      setChapter(data);
      setCurrentContent(data.content);
    } catch (err) {
      setError(err.message);
    } finally {
      setLoading(false);
    }
  }, [chapterId, preferences?.preferred_language]);

  useEffect(() => {
    if (chapterId) {
      loadChapter();
    }
  }, [chapterId, loadChapter]);

  // Set up text selection handler
  useEffect(() => {
    if (!ExecutionEnvironment.canUseDOM) {
      return;
    }

    const handleSelection = () => {
      const selectedTextObj = window.getSelection();
      const selectedText = selectedTextObj.toString().trim();

      if (selectedText) {
        // Only set if selection is substantial (more than 5 characters)
        if (selectedText.length > 5) {
          setSelectedText(selectedText);
        }
      }
    };

    document.addEventListener('mouseup', handleSelection);
    return () => {
      document.removeEventListener('mouseup', handleSelection);
    };
  }, []);

  // This function is no longer needed since we're using modal approach
  // const handleContentChange = (newContent) => {
  //   setCurrentContent(newContent);
  // };

  const handleDifficultyChange = (difficulty) => {
    // When difficulty changes, potentially reload content with new difficulty
    // This would trigger the personalization API call with the new difficulty
    loadChapter();
  };

  if (loading) {
    return (
      <div className="container margin-vert--xl">
        <div className="row">
          <div className="col col--8 col--offset-2">
            <div className="text--center padding-vert--lg">
              <h1>Loading chapter...</h1>
              <div className="loader">Loading...</div>
            </div>
          </div>
        </div>
      </div>
    );
  }

  if (error) {
    return (
      <div className="container margin-vert--xl">
        <div className="row">
          <div className="col col--8 col--offset-2">
            <div className="alert alert--danger">
              <h1>Error loading chapter</h1>
              <p>{error}</p>
            </div>
          </div>
        </div>
      </div>
    );
  }

  if (!chapter) {
    return (
      <div className="container margin-vert--xl">
        <div className="row">
          <div className="col col--8 col--offset-2">
            <div className="alert alert--warning">
              <h1>Chapter not found</h1>
            </div>
          </div>
        </div>
      </div>
    );
  }

  // Determine content direction based on language
  const contentDirection = isUrduContent() ? 'rtl' : 'ltr';

  return (
    <div className="container margin-vert--lg">
      <div className="row">
        {/* Chapter Content Panel */}
        <div className="col col--8">
          <article className="markdown">
            <header className="margin-bottom--lg">
              <h1 className="hero__title">{chapter.title}</h1>
            </header>

            {/* Chapter Controls */}
            <ChapterControls
              chapterId={chapterId}
              currentContent={currentContent}
              onDifficultyChange={handleDifficultyChange}
            />

            <div
              className="chapter-content"
              style={{
                lineHeight: '1.6',
                direction: contentDirection,
                fontFamily: isUrduContent() ? '"Noto Nastaliq Urdu", "Jameel Noori Nastaleeq", "Urdu Typesetting", serif' : 'inherit'
              }}
            >
              {/* Render chapter content - in a real implementation, you'd properly render markdown */}
              <div
                dangerouslySetInnerHTML={{
                  __html: currentContent.replace(/\n/g, '<br />')
                }}
              />
            </div>

            <section className="margin-vert--lg">
              <h2 className="text--secondary">Learning Outcomes</h2>
              <ul className="margin-left--lg">
                {chapter.learning_outcomes.map((outcome, index) => (
                  <li key={index}>{outcome}</li>
                ))}
              </ul>
            </section>
          </article>
        </div>

        {/* Chatbot Panel */}
        <div className="col col--4">
          <div className="card">
            <div className="card__header">
              <h3 className="card__title">Textbook Assistant</h3>
              {selectedText && (
                <div className="alert alert--info margin-top--sm">
                  <small>
                    <strong>Selected:</strong> "{selectedText.substring(0, 50)}{selectedText.length > 50 ? '...' : ''}"
                  </small>
                </div>
              )}
            </div>
            <div className="card__body">
              <RAGChatbot
                chapterId={chapterId}
                selectedText={selectedText}
                language={isUrduContent() ? 'ur' : 'en'}
              />
            </div>
          </div>
        </div>
      </div>
    </div>
  );
};

export default ChapterContent;