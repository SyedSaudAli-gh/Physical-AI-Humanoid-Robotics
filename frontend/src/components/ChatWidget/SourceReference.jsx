/**
 * SourceReference Component
 * Displays source references for chatbot responses
 */

import React from 'react';
import clsx from 'clsx';
import styles from './SourceReference.module.css';

const SourceReference = ({ source, className }) => {
  const { title, url, excerpt } = source;

  return (
    <div className={clsx(styles.sourceReference, className)} role="listitem">
      {url ? (
        <a
          href={url}
          target="_blank"
          rel="noopener noreferrer"
          className={styles.sourceLink}
          aria-label={title ? `Source: ${title} (opens in new tab)` : `Source: ${url} (opens in new tab)`}
        >
          <div className={styles.sourceTitle}>{title || url}</div>
          {excerpt && <div className={styles.sourceExcerpt}>{excerpt}</div>}
        </a>
      ) : (
        <div className={styles.sourceInfo} role="listitem">
          <div className={styles.sourceTitle}>{title}</div>
          {excerpt && <div className={styles.sourceExcerpt}>{excerpt}</div>}
        </div>
      )}
    </div>
  );
};

export default SourceReference;