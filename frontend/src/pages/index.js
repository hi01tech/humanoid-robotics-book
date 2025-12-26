import React from 'react';
import clsx from 'clsx';
import Layout from '@theme/Layout';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import styles from './index.module.css';
import useTranslate from '../hooks/useTranslate'; // Import the hook

// Import the hero image
import HeroBotImage from '@site/src/img/robot-landing.png';

import learningPathItems from '@site/static/learningPath.json';

import WhatsNew from '@site/src/components/WhatsNew';
import About from '@site/src/components/About';

function HeroSplit() {
  const { translatedText: translatedTitle } = useTranslate("Master Humanoid Robotics");
  const { translatedText: translatedSubtitle } = useTranslate(
    "A deep dive into the engineering, AI, and control systems of bipedal machines."
  );

  return (
    <header className={clsx('hero', styles.heroSplit)}>
      <div className={clsx('container', styles.heroContainer)}>
        {/* Left Column: Image */}
        <div className={styles.heroImageContainer}>
          <img
            src={HeroBotImage}
            alt="Hero Robot"
            className={styles.heroImage}
          />
        </div>

        {/* Right Column: Text and CTA */}
        <div className={styles.heroTextContainer}>
          <h1 className="hero__title">{translatedTitle}</h1>
          <p className="hero__subtitle">{translatedSubtitle}</p>
          <div>
            <Link
              className="button button--secondary button--lg " style={{ backgroundColor: '#aada8e' }}
              to= "/docs/introduction/week1"
            >
              Start Learning
            </Link>
          </div>
        </div>
      </div>
    </header>
  );
}

function LearningPath() {
  return (
    <section className={styles.learningPath}>
      <div className="container">
        <div className="row">
          {learningPathItems.map((item, idx) => (
            <div key={idx} className={clsx('col col--6', styles.cardContainer)}>
              <Link to={item.link} className={styles.cardLink}>
                <div className="card">
                  <div className="card__header">
                    <div className={styles.cardHeaderWithIcon}>
                      
                      <h3>{item.title}</h3>
                    </div>
                  </div>
                  <div className="card__body">
                    <p>{item.description}</p>
                    <h4 className={styles.learningOutcomesHeading}>
                      Learning Outcomes:
                    </h4>
                    <ul className={styles.learningOutcomesList}>
                      {item.learningOutcomes.map((outcome, outcomeIdx) => (
                        <li key={outcomeIdx}>{outcome}</li>
                      ))}
                    </ul>
                  </div>
                </div>
              </Link>
            </div>
          ))}
        </div>
      </div>
    </section>
  );
}

export default function Home() {
  const { siteConfig } = useDocusaurusContext();
  return (
    <Layout
      title={`Home`}
      description="A deep dive into the engineering, AI, and control systems of bipedal machines."
    >
      <HeroSplit />
      <main>
        <LearningPath />
        <WhatsNew />
        <About />
      </main>
    </Layout>
  );
}
