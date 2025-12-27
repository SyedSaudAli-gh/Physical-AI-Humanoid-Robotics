/**
 * ModuleCard component for the Physical AI & Humanoid Robotics textbook
 * Displays module information with title, description, and navigation link
 */

import React from 'react';
import './ModuleCard.css';

const ModuleCard = ({ id, title, description, icon, url, chapterCount }) => {
  return (
    <div className="module-card" id={id} role="listitem">
      <div className="module-card-content">
        <div className="module-icon" aria-hidden="true">
          {/* Using a generic approach for icons - could be replaced with actual icon implementation */}
          {icon ? (
            <i className={`icon ${icon}`} aria-hidden="true"></i>
          ) : (
            <i className="icon default-module-icon" aria-hidden="true"></i>
          )}
        </div>

        <h3 className="module-title">{title}</h3>

        <p className="module-description">{description}</p>

        <div className="module-meta">
          <span className="chapter-count" aria-label={`${chapterCount} chapters`}>
            {chapterCount} {chapterCount === 1 ? 'Chapter' : 'Chapters'}
          </span>
        </div>

        <a
          href={url}
          className="module-link"
          aria-label={`Explore ${title} module`}
          role="link"
        >
          View Module
          <span className="arrow" aria-hidden="true">→</span>
        </a>
      </div>
    </div>
  );
};

// Default props for the ModuleCard component
ModuleCard.defaultProps = {
  icon: null,
  chapterCount: 0
};

export default ModuleCard;