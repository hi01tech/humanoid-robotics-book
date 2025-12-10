import React, { useState } from 'react';
import Layout from '@theme/Layout';
import styles from './search.module.css';

function SearchPage() {
  const [query, setQuery] = useState('');
  const [result, setResult] = useState(null);
  const [isLoading, setIsLoading] = useState(false);
  const [error, setError] = useState(null);

  const handleSearch = async (e) => {
    e.preventDefault();
    if (!query) return;

    setIsLoading(true);
    setError(null);
    setResult(null);

    try {
      const response = await fetch('http://127.0.0.1:8000/api/v1/query', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({ question: query }),
      });

      if (!response.ok) {
        throw new Error(`HTTP error! Status: ${response.status}`);
      }

      const data = await response.json();
      setResult(data);
    } catch (err) {
      setError('Failed to fetch search results. Please ensure the backend server is running.');
      console.error(err);
    } finally {
      setIsLoading(false);
    }
  };

  return (
    <Layout title="Search" description="Search the Humanoid Robotics Textbook">
      <div className={styles.searchContainer}>
        <h1>Ask a question</h1>
        <p>Use the RAG search to ask a question about the content in this textbook.</p>
        <form onSubmit={handleSearch} className={styles.searchForm}>
          <input
            type="text"
            value={query}
            onChange={(e) => setQuery(e.target.value)}
            placeholder="e.g., What is a digital twin?"
            className={styles.searchInput}
            disabled={isLoading}
          />
          <button type="submit" className={styles.searchButton} disabled={isLoading}>
            {isLoading ? 'Searching...' : 'Search'}
          </button>
        </form>

        {error && <div className={styles.errorContainer}>{error}</div>}

        {result && (
          <div className={styles.resultsContainer}>
            <h2>Answer</h2>
            <p className={styles.answerText}>{result.answer}</p>

            <h3>Sources</h3>
            <ul className={styles.sourceList}>
              {result.sources.map((source, index) => (
                <li key={index} className={styles.sourceItem}>
                  <p className={styles.sourceContent}>"{source.content}"</p>
                  <p className={styles.sourceMeta}>
                    <strong>Source:</strong> {source.source}
                  </p>
                </li>
              ))}
            </ul>
          </div>
        )}
      </div>
    </Layout>
  );
}

export default SearchPage;
