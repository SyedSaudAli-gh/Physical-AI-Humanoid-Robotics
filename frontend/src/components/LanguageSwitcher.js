import React, { useState, useEffect } from 'react';
import { useUser } from '../contexts/UserContext';
import ExecutionEnvironment from '@docusaurus/ExecutionEnvironment';

// Language switcher component for the UI
const LanguageSwitcher = ({ onLanguageChange }) => {
  const { preferences, updatePreferences, loading } = useUser();
  const [selectedLanguage, setSelectedLanguage] = useState('en');

  useEffect(() => {
    // Initialize with user's preferred language
    if (preferences?.preferred_language) {
      setSelectedLanguage(preferences.preferred_language);
    }
  }, [preferences]);

  const handleLanguageChange = async (langCode) => {
    if (!ExecutionEnvironment.canUseDOM) {
      // Handle the case when DOM is not available (server-side rendering)
      setSelectedLanguage(langCode);
      return;
    }

    setSelectedLanguage(langCode);

    // Update user preferences
    await updatePreferences({ preferred_language: langCode });

    // Notify parent component of language change
    if (onLanguageChange) {
      onLanguageChange(langCode);
    }

    // In a real implementation, you might reload content with new language
    // window.location.reload(); // Only if needed based on your architecture
  };

  return (
    <div className="language-switcher">
      <label htmlFor="language-select" className="margin-right--sm">Language:</label>
      <select
        id="language-select"
        value={selectedLanguage}
        onChange={(e) => handleLanguageChange(e.target.value)}
        disabled={loading}
        className="form-select"
      >
        <option value="en">English</option>
        <option value="ur">Urdu</option>
      </select>
      
      {selectedLanguage === 'ur' && (
        <span className="badge badge--success margin-left--sm">اُردو</span>
      )}
    </div>
  );
};

export default LanguageSwitcher;