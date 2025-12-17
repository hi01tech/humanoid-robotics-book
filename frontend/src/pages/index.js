import React from 'react';
import clsx from 'clsx';
import Layout from '@theme/Layout';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import styles from './index.module.css';

// Import the hero image
import HeroBotImage from '@site/src/img/robot-landing.png';

const learningPathItems = [
  
  {
    title: 'Module 1: ROS 2 Ecosystem',
    description: 'Master the core concepts of the Robot Operating System for complex applications.',
    learningOutcomes: [
      'Navigate ROS 2 communication patterns',
      'Develop Python-based ROS 2 nodes',
      'Utilize launch files and parameters',
    ],
    link: '/docs/ros2/week4',
  },
  {
    title: 'Module 2: Simulation & Digital Twins',
    description: 'Use NVIDIA Isaac Sim to create photorealistic, physically-accurate digital twins.',
    learningOutcomes: [
      'Import robot models into Isaac Sim',
      'Configure simulated sensors',
      'Integrate with ros2_control',
    ],
    link: '/docs/digital-twin/week6',
  },
  {
    title: 'Module 3: The AI-Robot Brain (NVIDIA Isaac™)',
    description: 'Build scalable robotics simulation application and synthetic data generation.',
    learningOutcomes: [
      'Master kinematics and dynamics',
      'Understand real-time control principles',
      'Implement sensor fusion techniques',
    ],
    link: '/docs/isaac-sim/week8  ',
  },
  {
    title: 'Module 4: Vision-Language-Action (VLA)',
    description: 'Implement advanced perception and decision-making with Isaac ROS and VLAs.',
    learningOutcomes: [
      'Accelerate perception with Isaac ROS',
      'Implement SLAM and object detection',
      'Explore Vision-Language-Action models',
    ],
    link: '/docs/vla/vla-week11',
  },
];

function HeroSplit() {
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
          <h1 className="hero__title">Master Humanoid Robotics</h1>
          <p className="hero__subtitle">
            A deep dive into the engineering, AI, and control systems of
            bipedal machines.
          </p>
          <div>
            <Link
              className="button button--secondary button--lg " style={{ backgroundColor: '#aada8e' }}
              to="/docs/intro"
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
                    <h3>{item.title}</h3>
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
      </main>
    </Layout>
  );
}
