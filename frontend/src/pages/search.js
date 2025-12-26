import React, { useState } from 'react';
import Layout from '@theme/Layout';
import styles from './search.module.css';

export default function SearchPage() {
  const [query, setQuery] = useState('');
  const [result, setResult] = useState(null);
  const [isLoading, setIsLoading] = useState(false);
  const [error, setError] = useState(null);

      const handleSearch = async () => {
      if (!query) return;
  
      setIsLoading(true);
      setError(null);
      setResult(null);
  
      const targetLanguage = localStorage.getItem('docusaurus.language') || 'en'; // Get target language from localStorage
  
      try {
        const response = await fetch('http://127.0.0.1:8000/rag/query', {
          method: 'POST',
          headers: {
            'Content-Type': 'application/json',
          },
          body: JSON.stringify({ question: query, top_k: 3, target_language: targetLanguage }), // Pass target language
        });
      if (!response.ok) {
        throw new Error(`HTTP error! status: ${response.status}`);
      }

      const data = await response.json();
      setResult(data);
    } catch (e) {
      setError(`Failed to fetch search results: ${e.message}`);
    } finally {
      setIsLoading(false);
    }
  };

  return (
    <Layout title="Search" description="Search the textbook content">
      <div className={styles.searchContainer}>
        <h1>Ask a question about the textbook</h1>
        <div className={styles.inputContainer}>
          <input
            type="text"
            value={query}
            onChange={(e) => setQuery(e.target.value)}
            onKeyPress={(e) => e.key === 'Enter' && handleSearch()}
            className={styles.searchInput}
            placeholder="What is Visual SLAM?"
          />
          <button
            onClick={handleSearch}
            className="button button--primary"
            disabled={isLoading}
          >
            {isLoading ? 'Searching...' : 'Search'}
          </button>
        </div>

        {error && <div className={styles.errorContainer}>{error}</div>}

        {result && (
          <div className={styles.resultsContainer}>
            <h2>Answer</h2>
            <p className={styles.answerText}>{result.answer}</p>

            <h3>Sources</h3>
            <div className={styles.sourcesContainer}>
              {result.sources.map((source, index) => (
                <div key={index} className={styles.sourceChunk}>
                  {source.url ? (
                    <a href={source.url} target="_blank" rel="noopener noreferrer">
                      <p className={styles.sourceTitle}>{source.source}</p>
                    </a>
                  ) : (
                    <p className={styles.sourceTitle}>{source.source}</p>
                  )}
                  <p>{source.content}</p>
                </div>
              ))}
            </div>
          </div>
        )}
      </div>
    </Layout>
  );
}