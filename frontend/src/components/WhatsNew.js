import React from 'react';
import styles from './WhatsNew.module.css';
import Link from '@docusaurus/Link';

const recentUpdates = [
  {
    title: 'New Chapter: Advanced Control Systems',
    date: '2025-12-20',
    description: 'A deep dive into modern control theory and its application to humanoid robots.',
    link: '/docs/introduction/week3', // Placeholder link
  },
  {
    title: 'Updated: ROS 2 Quick Reference Guide',
    date: '2025-12-15',
    description: 'Added new commands and updated existing ones for ROS 2 Iron.',
    link: '/docs/reference/ros2-quick-reference',
  },
  {
    title: 'New Tutorial: Building a Digital Twin',
    date: '2025-12-10',
    description: 'Learn how to create a digital twin of your robot in NVIDIA Isaac Sim.',
    link: '/docs/digital-twin/week6', // Placeholder link
  },
];

export default function WhatsNew() {
  return (
    <section className={styles.whatsNew}>
      <div className="container">
        <h2>What's New</h2>
        <div className={styles.updatesContainer}>
          {recentUpdates.map((update, index) => (
            <div key={index} className={styles.updateItem}>
              <h3>
                <Link to={update.link}style={{ color: '#327cd1ff' }}> {update.title}</Link>
              </h3>
              <p className={styles.updateDate}>{update.date}</p>
              <p>{update.description}</p>
            </div>
          ))}
        </div>
      </div>
    </section>
  );
}
