import React, { useState, useEffect } from 'react';
import { useUser } from '../contexts/UserContext';
import styles from './UrduTranslation.module.css';

// Urdu translation button component with modal
const UrduTranslateButton = ({
  content,
  isLoggedIn = true
}) => {
  const [isTranslating, setIsTranslating] = useState(false);
  const [isModalOpen, setIsModalOpen] = useState(false);
  const [translatedText, setTranslatedText] = useState('');
  const [error, setError] = useState(null);
  const { user, loading } = useUser();

  // Only show the button if the user is logged in
  if (!isLoggedIn || !user) {
    return null;
  }

  const handleTranslate = async () => {
    if (!content || isTranslating || loading) return;

    setIsTranslating(true);
    setError(null);

    try {
      // Call the backend translation API
      const response = await fetch('http://localhost:8000/translate-urdu', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          text: content
        })
      });

      if (!response.ok) {
        throw new Error(`HTTP error! status: ${response.status}`);
      }

      const data = await response.json();
      setTranslatedText(data.urdu_text);
      setIsModalOpen(true);
    } catch (error) {
      console.error('Translation error:', error);
      setError('ترجمہ میں مسئلہ، دوبارہ کوشش کریں');
      alert('ترجمہ میں مسئلہ، دوبارہ کوشش کریں');
    } finally {
      setIsTranslating(false);
    }
  };

  const closeModal = () => {
    setIsModalOpen(false);
    setTranslatedText('');
  };

  // Handle Escape key to close modal
  useEffect(() => {
    const handleEscKey = (e) => {
      if (e.key === 'Escape' && isModalOpen) {
        closeModal();
      }
    };

    window.addEventListener('keydown', handleEscKey);
    return () => window.removeEventListener('keydown', handleEscKey);
  }, [isModalOpen]);

  return (
    <div className={styles.translationButtonContainer}>
      <button
        className={`button button--${isTranslating ? 'secondary' : 'primary'}`}
        onClick={handleTranslate}
        disabled={isTranslating || loading}
      >
        {isTranslating ? (
          <>
            <span className="loading-spinner loading-spinner--sm"></span> ترجمہ ہو رہا ہے...
          </>
        ) : (
          'اردو میں ترجمہ'
        )}
      </button>

      {/* Modal for displaying translated text */}
      {isModalOpen && (
        <div className={styles.modalOverlay} onClick={closeModal}>
          <div className={styles.modalContent} onClick={(e) => e.stopPropagation()}>
            <div className={styles.modalHeader}>
              <h3 className={styles.modalTitle}>ترجمہ شدہ مواد</h3>
              <button
                className={styles.closeButton}
                onClick={closeModal}
                aria-label="Close modal"
              >
                ×
              </button>
            </div>
            <div className={styles.modalBody}>
              <div
                className={styles.urduContent}
                dangerouslySetInnerHTML={{ __html: translatedText.replace(/\n/g, '<br />') }}
              />
            </div>
            <div className={styles.modalFooter}>
              <button
                className="button button--primary"
                onClick={closeModal}
              >
                انگریزی میں واپس
              </button>
            </div>
          </div>
        </div>
      )}
    </div>
  );
};

export default UrduTranslateButton;