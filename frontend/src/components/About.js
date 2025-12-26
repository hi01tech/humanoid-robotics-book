import React from 'react';
import styles from './About.module.css';
import Link from '@docusaurus/Link';

export default function About() {
  return (
    <section className={styles.about}>
      <div className="container">
        <div className={styles.aboutContainer}>
          <div className={styles.aboutColumn}>
            <h2>About the Author</h2>
            <p>
              This textbook is written and maintained by a passionate roboticists and AI developer dedicated to making humanoid robotics accessible to everyone.
            </p>
          </div>
          <div className={styles.aboutColumn}>
            <h2>Contribute</h2>
            <p>
              This is a community-driven project. We welcome contributions of all kinds, from typo fixes to new chapters.
            </p>
            <Link
              className="button button--secondary"
              to="https://github.com/hi01tech/humanoid-robotics-book"
            >
              Get Involved
            </Link>
          </div>
        </div>
      </div>
    </section>
  );
}
