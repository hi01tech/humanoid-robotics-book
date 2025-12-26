// frontend/src/hooks/useTranslate.js
import { useState, useEffect } from 'react';

const useTranslate = (text) => {
  const [translatedText, setTranslatedText] = useState(text);
  const [isLoading, setIsLoading] = useState(false);
  const [error, setError] = useState(null);

  useEffect(() => {
    // Set up a timer to debounce requests
    const handler = setTimeout(() => {
      const translate = async () => {
        const targetLanguage = localStorage.getItem('docusaurus.language') || 'en';

        if (targetLanguage === 'en') {
          setTranslatedText(text);
          return;
        }

        if (!text) {
          setTranslatedText('');
          return;
        }

        setIsLoading(true);
        setError(null);

        try {
          const response = await fetch('http://127.0.0.1:8000/api/translate-text', {
            method: 'POST',
            headers: {
              'Content-Type': 'application/json',
            },
            body: JSON.stringify({ text, target_language: targetLanguage }),
          });

          if (!response.ok) {
            throw new Error(`HTTP error! status: ${response.status}`);
          }

          const data = await response.json();
          setTranslatedText(data.translated_text);
        } catch (e) {
          setError(`Translation failed: ${e.message}`);
          setTranslatedText(text); // Fallback to original text on error
        } finally {
          setIsLoading(false);
        }
      };

      translate();
    }, 500); // 500ms debounce delay

    // Cleanup function to cancel the timer if the text changes
    return () => {
      clearTimeout(handler);
    };
  }, [text]); // Re-run effect if the original text changes

  return { translatedText, isLoading, error };
};

export default useTranslate;
