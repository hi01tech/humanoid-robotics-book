import React, { useState, useRef, useEffect } from 'react';
import Layout from '@theme/Layout';
import styles from './chatbot.module.css';
import { useLocation } from '@docusaurus/router';

const BOT_NAME = "Humanoid Robotics Assistant";

export default function ChatbotPage() {
  const [messages, setMessages] = useState([]);
  const [inputMessage, setInputMessage] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [error, setError] = useState(null);
  const messagesEndRef = useRef(null);
  const location = useLocation();

  // Scroll to bottom of messages whenever they update
  useEffect(() => {
    messagesEndRef.current?.scrollIntoView({ behavior: "smooth" });
  }, [messages]);

  // Initial message when the component mounts or URL changes (e.g., first visit)
  useEffect(() => {
    if (messages.length === 0) {
      setMessages([
        { type: 'bot', text: `Hello! I'm your ${BOT_NAME}. Ask me anything about the textbook!`}
      ]);
    }
  }, [location.pathname]); // Re-run if path changes (e.g. user navigates away and back)


      const handleSendMessage = async () => {
      if (!inputMessage.trim()) return;
  
      const userMessage = { type: 'user', text: inputMessage };
      setMessages(prevMessages => [...prevMessages, userMessage]);
      setInputMessage('');
      setIsLoading(true);
      setError(null);
  
      const targetLanguage = localStorage.getItem('docusaurus.language') || 'en'; // Get target language from localStorage
  
      try {
        const response = await fetch('http://127.0.0.1:8000/rag/query', {
          method: 'POST',
          headers: {
            'Content-Type': 'application/json',
          },
          body: JSON.stringify({
            question: userMessage.text,
            top_k: 3,
            target_language: targetLanguage, // Pass target language
          }),
        });
      if (!response.ok) {
        throw new Error(`HTTP error! status: ${response.status}`);
      }

      const data = await response.json();
      const botResponse = {
        type: 'bot',
        text: data.answer,
        sources: data.sources // Assuming sources are part of the data
      };
      setMessages(prevMessages => [...prevMessages, botResponse]);

    } catch (e) {
      setError(`Failed to get a response: ${e.message}`);
      setMessages(prevMessages => [...prevMessages, { type: 'bot', text: `Sorry, I encountered an error: ${e.message}`}]);
    } finally {
      setIsLoading(false);
    }
  };

  const renderMessageContent = (message) => {
    if (message.type === 'bot' && message.sources && message.sources.length > 0) {
      return (
        <>
          <p>{message.text}</p>
          <div className={styles.sourcesContainer}>
            <strong>Sources:</strong>
            <ul>
              {message.sources.map((source, index) => (
                <li key={index}>
                  {source.url ? (
                    <a href={source.url} target="_blank" rel="noopener noreferrer">
                      {source.source.split('/').pop().replace('.md', '').replace(/[-_]/g, ' ')}
                    </a>
                  ) : (
                    source.source.split('/').pop().replace('.md', '').replace(/[-_]/g, ' ')
                  )}
                </li>
              ))}
            </ul>
          </div>
        </>
      );
    }
    return <p>{message.text}</p>;
  };

  return (
    <Layout title="Chat" description="Chat with the Humanoid Robotics Assistant">
      <div className={styles.chatbotContainer}>
        <h1 className={styles.chatTitle}>Chat with {BOT_NAME}</h1>
        <div className={styles.messageArea}>
          {messages.map((message, index) => (
            <div key={index} className={message.type === 'user' ? styles.userMessage : styles.botMessage}>
              {renderMessageContent(message)}
            </div>
          ))}
          <div ref={messagesEndRef} />
          {isLoading && <div className={styles.loadingIndicator}>Thinking...</div>}
          {error && <div className={styles.errorContainer}>{error}</div>}
        </div>
        <div className={styles.inputArea}>
          <input
            type="text"
            value={inputMessage}
            onChange={(e) => setInputMessage(e.target.value)}
            onKeyPress={(e) => e.key === 'Enter' && handleSendMessage()}
            className={styles.chatInput}
            placeholder="Ask me anything about humanoid robotics..."
            disabled={isLoading}
          />
          <button onClick={handleSendMessage} className="button button--primary" disabled={isLoading}>
            Send
          </button>
        </div>
      </div>
    </Layout>
  );
}
