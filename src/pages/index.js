import React from 'react';
import clsx from 'clsx';
import Layout from '@theme/Layout';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import styles from './index.module.css'; // Assuming custom CSS for styling

// Define module data for cards
const moduleData = [
  {
    title: 'Module 1: ROS 2',
    weeks: 'Weeks 4-5',
    learningOutcomes: ['Core ROS 2 concepts', 'Launch files', 'Parameters'],
    link: '/docs/ros2/week4', // Link to the first week of the module
  },
  {
    title: 'Module 2: Digital Twin',
    weeks: 'Weeks 6-7',
    learningOutcomes: ['Isaac Sim introduction', 'Robot simulation', 'ros2_control'],
    link: '/docs/digital-twin/week6',
  },
  {
    title: 'Module 3: NVIDIA Isaac',
    weeks: 'Weeks 8-10',
    learningOutcomes: ['Isaac ROS basics', 'SLAM', 'Object Detection'],
    link: '/docs/isaac-sim/week8',
  },
  {
    title: 'Module 4: VLA',
    weeks: 'Weeks 11-13',
    learningOutcomes: ['VLA architectures', 'Policy learning', 'Advanced perception'],
    link: '/docs/vla/vla-week11', // Corrected link
  },
];

// Quick Links data
const quickLinks = [
  {
    label: 'Setup Guides',
    link: '/docs/setup-guides', // Corrected link
  },
  {
    label: 'Assessments',
    link: '/docs/assessments', 
  },
  {
    label: 'Glossary',
    link: '/docs/reference/glossary',
  },
];

function HomepageHeader() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <header className={clsx('hero hero--primary', styles.heroBanner)}>
      <div className="container">
        <h1 className="hero__title">{siteConfig.title}</h1>
        <p className="hero__subtitle">{siteConfig.tagline}</p>
      </div>
    </header>
  );
}

function ModuleCard({ title, weeks, learningOutcomes, link }) {
  return (
    <div className={clsx('col col--3', styles.moduleCard)}>
      <div className="card">
        <div className="card__header">
          <h3>{title}</h3>
        </div>
        <div className="card__body">
          <p className={styles.moduleWeeks}>{weeks}</p>
          <ul className={styles.learningOutcomes}>
            {learningOutcomes.map((outcome, idx) => (
              <li key={idx}>{outcome}</li>
            ))}
          </ul>
        </div>
        <div className="card__footer">
          <Link className="button button--primary button--block" to={link}>
            Go to Module
          </Link>
        </div>
      </div>
    </div>
  );
}

function QuickLinksSidebar() {
  return (
    <div className={clsx('col col--2', styles.quickLinksSidebar)}>
      <div className="card">
        <div className="card__header">
          <h3>Quick Links</h3>
        </div>
        <div className="card__body">
          <ul className={styles.quickLinksList}>
            {quickLinks.map((item, idx) => (
              <li key={idx}>
                <Link to={item.link}>{item.label}</Link>
              </li>
            ))}
          </ul>
        </div>
      </div>
    </div>
  );
}


export default function Home() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout
      title={`Hello from ${siteConfig.title}`}
      description="Description will go into a meta tag in <head />">
      <HomepageHeader />
      <main>
        <section className={styles.features}>
          <div className="container">
            <div className="row">
              <div className={clsx('col col--10')}>
                <div className="row">
                  {moduleData.map((props, idx) => (
                    <ModuleCard key={idx} {...props} />
                  ))}
                </div>
              </div>
              <QuickLinksSidebar />
            </div>
          </div>
        </section>
      </main>
    </Layout>
  );
}