import React, { useState, useEffect } from 'react';
import { useLocation, useHistory, Link } from '@docusaurus/router';
import { fetchModuleChapters, fetchChapterContent } from '../services/api';

// Component for navigating between chapters within a module
const ChapterNavigation = ({ moduleId, currentChapterId }) => {
  const [chapters, setChapters] = useState([]);
  const [currentChapter, setCurrentChapter] = useState(null);
  const [loading, setLoading] = useState(true);
  const [error, setError] = useState(null);
  
  const location = useLocation();
  const history = useHistory();

  useEffect(() => {
    const loadChapters = async () => {
      try {
        setLoading(true);
        const moduleChapters = await fetchModuleChapters(moduleId);
        setChapters(moduleChapters);
        
        if (currentChapterId) {
          // Load the current chapter content
          const chapterContent = await fetchChapterContent(currentChapterId);
          setCurrentChapter(chapterContent);
        }
        setLoading(false);
      } catch (err) {
        setError(err.message);
        setLoading(false);
      }
    };

    if (moduleId) {
      loadChapters();
    }
  }, [moduleId, currentChapterId]);

  if (loading) {
    return <div>Loading chapter list...</div>;
  }

  if (error) {
    return <div>Error loading chapters: {error}</div>;
  }

  // Find current chapter index for navigation
  const currentIndex = chapters.findIndex(ch => ch.id === currentChapterId);
  const prevChapter = currentIndex > 0 ? chapters[currentIndex - 1] : null;
  const nextChapter = currentIndex < chapters.length - 1 ? chapters[currentIndex + 1] : null;

  return (
    <div className="chapter-navigation">
      <div className="navigation-header">
        <h3>Chapters in Module</h3>
      </div>
      
      <ul className="chapter-list">
        {chapters.map((chapter, index) => (
          <li 
            key={chapter.id} 
            className={`chapter-item ${chapter.id === currentChapterId ? 'current' : ''}`}
          >
            <Link 
              to={`/docs/${location.pathname.split('/')[2]}/${chapter.id}`}
              className={`chapter-link ${chapter.id === currentChapterId ? 'active' : ''}`}
            >
              <span className="chapter-index">{index + 1}.</span>
              <span className="chapter-title">{chapter.title}</span>
              {chapter.id === currentChapterId && <span className="current-indicator">(current)</span>}
            </Link>
          </li>
        ))}
      </ul>
      
      <div className="chapter-nav-buttons">
        {prevChapter && (
          <Link to={`/docs/${location.pathname.split('/')[2]}/${prevChapter.id}`} className="button button--secondary">
            ← Previous: {prevChapter.title}
          </Link>
        )}
        
        {nextChapter && (
          <Link to={`/docs/${location.pathname.split('/')[2]}/${nextChapter.id}`} className="button button--primary float-right">
            Next: {nextChapter.title} →
          </Link>
        )}
      </div>
    </div>
  );
};

export default ChapterNavigation;