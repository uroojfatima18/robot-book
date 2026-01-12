import React, { useEffect, useState } from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import styles from './index.module.css';

// Feature Data
const FEATURES = [
  {
    title: 'Robotics Fundamentals',
    description: 'Learn the core principles of robot design, kinematics, and control systems.',
    icon: '🤖',
    link: '/docs/01-ros2-nervous-system/introduction'
  },
  {
    title: 'Programming & Control',
    description: 'Master ROS 2, Python, and C++ for building intelligent robotic systems.',
    icon: '💻',
    link: '/docs/02-digital-twin/introduction'
  },
  {
    title: 'AI & Machine Learning',
    description: 'Integrate computer vision, deep learning, and autonomous decision-making.',
    icon: '🧠',
    link: '/docs/03-ai-robot-brain/introduction'
  },
];

function TechCard({title, description, icon, link}) {
  return (
    <div className={clsx('col col--4')}>
      <Link to={link || '#'} className={styles.cardLink}>
        <div className="tech-card padding--lg margin-bottom--lg h-100">
          <div className="text--center tech-card-icon">
            {icon}
          </div>
          <div className="text--center padding-horiz--md">
            <h3>{title}</h3>
            <p>{description}</p>
          </div>
        </div>
      </Link>
    </div>
  );
}

function HomepageHeader() {
  const {siteConfig} = useDocusaurusContext();

  return (
    <header className={styles.heroSection}>
       <div className="container">
        <div className="row">
          {/* Left Content */}
          <div className={clsx('col col--6', styles.heroTextCol)}>
            <h1 className={styles.heroTitle}>
              <span className={styles.mustardText}>PHYSICAL</span>{' '}
              <span className={styles.blueGlow}>AI &</span>
              <br />
              <span className={styles.mustardText}>HUMANOID</span>
              <br />
              <span className={styles.goldText}>ROBOTS</span>
            </h1>
            
            <p className={styles.heroDescription}>
              The definitive technical guide to building intelligent machines. 
              From neural architectures to dynamic locomotion.
            </p>

            <div className={styles.statsRow}>
              <div className={styles.statItem}>
                <span>CHAPTERS</span>
              </div>
              <div className={styles.statItem}>
                <span>DIAGRAMS</span>
              </div>
              <div className={styles.statItem}>
                <span>CODE EXAMPLES</span>
              </div>
            </div>

            <div className={styles.heroButtons}>
              <Link
                className="button button--primary button--lg margin-right--md"
                to="/docs/01-ros2-nervous-system/introduction">
                Begin Learning
              </Link>
              <Link
                className="button button--secondary button--lg button--outline"
                to="/docs/introduction">
                View Contents
              </Link>
            </div>
          </div>

          {/* Right Image */}
          <div className={clsx('col col--6', styles.heroImageCol)}>
            <div className={styles.imageContainer}>
              <img 
                src="img/hero-robot.png" 
                alt="Humanoid Robot" 
                className={styles.heroImage}
              />
              {/* Optional: We could add animated HTML overlays here if the image didn't have them */}
            </div>
          </div>
        </div>
      </div>
    </header>
  );
}

import LearningPath from '@site/src/components/LearningPath';

export default function Home() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout
      title={`Home`}
      description="Physical AI & Humanoid Robots Book">
      <HomepageHeader />
      <main className={styles.mainContainer}>
        <LearningPath />
        <div className="container margin-vert--xl">
          <div className="row">
            {FEATURES.map((props, idx) => (
              <TechCard key={idx} {...props} />
            ))}
          </div>
        </div>
      </main>
    </Layout>
  );
}
