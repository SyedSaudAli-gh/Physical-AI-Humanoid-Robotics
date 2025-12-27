/**
 * BookLayout Component
 * Implements a book-style reading layout for chapter content
 */

import React from 'react';
import clsx from 'clsx';
import styles from './BookLayout.module.css';

const BookLayout = ({ children, title, authors, section, className }) => {
  return (
    <div className={clsx(styles.bookLayout, className)}>
      <header className={styles.bookHeader} role="banner">
        <div className={styles.bookTitleSection}>
          {title && <h1 className={styles.bookTitle}>{title}</h1>}
          {authors && (
            <div className={styles.bookAuthors}>
              {Array.isArray(authors)
                ? authors.map((author, index) => (
                    <span key={index} className={styles.author}>
                      {author}
                      {index < authors.length - 1 && ', '}
                    </span>
                  ))
                : <span className={styles.author}>{authors}</span>
              }
            </div>
          )}
        </div>
      </header>

      <div className={styles.bookContent}>
        {section && (
          <nav className={styles.bookNavigation} aria-label="Section navigation">
            <div className={styles.sectionInfo}>
              <span className={styles.sectionLabel}>Section:</span>
              <span className={styles.sectionValue}>{section}</span>
            </div>
          </nav>
        )}

        <main className={styles.chapterContent} role="main">
          {children}
        </main>
      </div>

      <footer className={styles.bookFooter} role="contentinfo">
        <div className={styles.pageNumber}>© Physical AI & Humanoid Robotics Textbook</div>
      </footer>
    </div>
  );
};

export default BookLayout;