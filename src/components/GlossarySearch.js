import React, { useState, useEffect } from 'react';
import styles from './GlossarySearch.module.css';

export default function GlossarySearch() {
  const [allTerms, setAllTerms] = useState([]);
  const [filteredTerms, setFilteredTerms] = useState([]);
  const [query, setQuery] = useState('');

  useEffect(() => {
    // The base URL is relative to the static directory
    fetch('/humanoid-robotics-book/glossary.json')
      .then(response => {
        if (!response.ok) {
          throw new Error('Network response was not ok');
        }
        return response.json();
      })
      .then(data => {
        setAllTerms(data);
        setFilteredTerms(data);
      })
      .catch(error => {
        console.error("Error fetching glossary:", error);
      });
  }, []);

  useEffect(() => {
    if (!query) {
      setFilteredTerms(allTerms);
      return;
    }

    const lowerCaseQuery = query.toLowerCase();
    const filtered = allTerms.filter(item => 
      item.term.toLowerCase().includes(lowerCaseQuery) ||
      item.definition.toLowerCase().includes(lowerCaseQuery)
    );
    setFilteredTerms(filtered);
  }, [query, allTerms]);

  return (
    <div>
      <input
        type="text"
        className={styles.glossarySearchInput}
        placeholder="Search glossary terms..."
        value={query}
        onChange={(e) => setQuery(e.target.value)}
      />
      <div className={styles.glossaryList}>
        {filteredTerms.map((item, index) => (
          <div key={index} className={styles.glossaryItem}>
            <h3>{item.term}</h3>
            <p>{item.definition}</p>
          </div>
        ))}
        {filteredTerms.length === 0 && query && (
            <p>No terms found for "{query}".</p>
        )}
      </div>
    </div>
  );
}
