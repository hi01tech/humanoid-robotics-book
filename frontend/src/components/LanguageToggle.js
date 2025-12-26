// frontend/src/components/LanguageToggle.js
import React, { useState, useEffect } from 'react';
import clsx from 'clsx';
import { useLocation } from '@docusaurus/router';
import styles from './LanguageToggle.module.css';

const SUPPORTED_LANGUAGES = [
  { code: 'en', label: 'English' },
  { code: 'ur', label: 'اردو' }, // Urdu
];

const LOCAL_STORAGE_KEY = 'docusaurus.language';

export default function LanguageToggle() {
  const location = useLocation();
  const [selectedLanguage, setSelectedLanguage] = useState('en'); // Default to English

  useEffect(() => {
    // Initialize from local storage
    const storedLang = localStorage.getItem(LOCAL_STORAGE_KEY);
    if (storedLang && SUPPORTED_LANGUAGES.some(lang => lang.code === storedLang)) {
      setSelectedLanguage(storedLang);
    } else {
      localStorage.setItem(LOCAL_STORAGE_KEY, 'en');
    }
  }, []);

  const handleLanguageChange = (event) => {
    const newLang = event.target.value;
    setSelectedLanguage(newLang);
    localStorage.setItem(LOCAL_STORAGE_KEY, newLang);
    // For now, a simple page reload to apply changes,
    // later we might integrate with Docusaurus's i18n
    window.location.reload();
  };

  return (
    <div className={clsx('navbar__item', styles.languageToggle)}>
      <select
        className={styles.languageSelect}
        onChange={handleLanguageChange}
        value={selectedLanguage}>
        {SUPPORTED_LANGUAGES.map((lang) => (
          <option key={lang.code} value={lang.code}>
            {lang.label}
          </option>
        ))}
      </select>
    </div>
  );
}
