import React, { useState, useEffect, useCallback } from 'react';
import { fetchChapterContent } from '../services/api';
import ChapterControls from './ChapterControls';
import { useUser } from '../contexts/UserContext';
import ExecutionEnvironment from '@docusaurus/ExecutionEnvironment';

// Component for displaying chapter content with new chat widget
const ChapterContent = ({ chapterId }) => {
  const [chapter, setChapter] = useState(null);
  const [currentContent, setCurrentContent] = useState('');
  const [loading, setLoading] = useState(true);
  const [error, setError] = useState(null);
  const [selectedText, setSelectedText] = useState('');
  const { preferences } = useUser();

  const loadChapter = useCallback(async () => {
    try {
      setLoading(true);
      const data = await fetchChapterContent(chapterId);
      setChapter(data);
      setCurrentContent(data.content);
    } catch (err) {
      setError(err.message);
    } finally {
      setLoading(false);
    }
  }, [chapterId]);

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
              onContentUpdate={setCurrentContent}
            />

            <div
              className="chapter-content"
              style={{
                lineHeight: '1.6',
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

        {/* Chatbot Panel - Now handled by the floating ChatWidget in Root.js */}
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
              <p className="text--center text--secondary">
                The interactive assistant is available as a floating button on the page.
              </p>
            </div>
          </div>
        </div>
      </div>
    </div>
  );
};

export default ChapterContent;