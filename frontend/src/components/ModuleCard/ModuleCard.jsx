/**
 * ModuleCard Component
 * Displays a module card with title, description, and link to the chapter
 */

import React from 'react';
import clsx from 'clsx';
import styles from './ModuleCard.module.css';

const ModuleCard = ({ id, title, description, chapterLink, icon, className }) => {
  return (
    <div className={clsx(styles.moduleCard, className)}>
      <div className={styles.cardHeader}>
        {icon && <div className={styles.cardIcon}>{icon}</div>}
        <h3 className={styles.cardTitle}>{title}</h3>
      </div>
      
      <div className={styles.cardBody}>
        <p className={styles.cardDescription}>{description}</p>
      </div>
      
      <div className={styles.cardFooter}>
        <a 
          href={chapterLink} 
          className={styles.cardLink}
          aria-label={`Go to ${title} module`}
        >
          Explore Module
          <span className={styles.arrow}>→</span>
        </a>
      </div>
    </div>
  );
};

export default ModuleCard;