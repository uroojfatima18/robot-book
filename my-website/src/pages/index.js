import React from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import Layout from '@theme/Layout';
import useBaseUrl from '@docusaurus/useBaseUrl';
import styles from './index.module.css';
import LearningPath from '@site/src/components/LearningPath';

const FEATURES = [
  {
    title: 'Robotics Fundamentals',
    description: 'Learn the core principles of robot design, kinematics, and control systems.',
    icon: '🤖',
    link: '/docs/01-ros2-nervous-system/chapter_1_introduction'
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

function StatusBlock() {
  return (
    <div className="status-bar">
      <div><span className="status-dot"></span>SYSTEM_ACTIVE</div>
      <div>NODE_01: ONLINE</div>
      <div>LATENCY: 12ms</div>
    </div>
  );
}

function TechCard({title, description, icon, link, index}) {
  return (
    <div className={clsx('col col--4')}>
      <Link to={link || '#'} style={{textDecoration: 'none', color: 'inherit'}}>
        <div className="tech-card padding--lg margin-bottom--lg h-100" style={{position: 'relative'}}>
          {/* Huge translucent number */}
          <div style={{
            position: 'absolute', 
            top: '10px', 
            right: '20px', 
            fontSize: '4rem', 
            fontWeight: '900', 
            color: 'rgba(138, 235, 255, 0.08)', 
            fontFamily: '"Space Grotesk", sans-serif',
            pointerEvents: 'none'
          }}>
            0{index + 1}
          </div>
          
          <div className="text--center tech-card-icon" style={{fontSize: '3rem', marginBottom: '1rem', position: 'relative', zIndex: 1}}>
            {icon}
          </div>
          <div className="text--center" style={{position: 'relative', zIndex: 1}}>
            <h3 style={{color: '#8aebff', fontSize: '1.2rem'}}>{title}</h3>
            <p style={{fontSize: '0.9rem', opacity: 0.8}}>{description}</p>
          </div>
        </div>
      </Link>
    </div>
  );
}

function HomepageHeader() {
  return (
    <header className={styles.heroSection}>
       <div className="container">
        <div className="row">
          <div className={clsx('col col--7', styles.heroTextCol)}>
            <StatusBlock />
            <h1 className={styles.heroTitle} style={{fontSize: '5rem', marginBottom: '1rem'}}>
              PHYSICAL<br/>
              <span style={{color: '#8aebff'}}>AI & ROBOTS</span>
            </h1>
            
            <p className={styles.heroDescription} style={{fontSize: '1.3rem', maxWidth: '600px'}}>
              The definitive technical guide to building intelligent machines. 
              From neural architectures to dynamic locomotion.
            </p>

            <div className={styles.heroButtons} style={{marginTop: '3rem'}}>
              <Link
                className="button button--primary button--lg margin-right--md"
                to="/docs/01-ros2-nervous-system/chapter_1_introduction">
                BEGIN LEARNING
              </Link>
              <Link
                className="button button--secondary button--outline button--lg"
                to="/docs/01-ros2-nervous-system/chapter_1_introduction">
                SYSTEM_SPECS
              </Link>
            </div>
          </div>

          <div className={clsx('col col--5', styles.heroImageCol)}>
            <div className={styles.imageContainer}>
              <div className={styles.hudNode} style={{top: '10%', left: '10%'}}>EYE_V_01: ONLINE</div>
              <div className={styles.hudNode} style={{top: '40%', right: '5%'}}>CORE_TEMP: 32°C</div>
              <div className={styles.hudNode} style={{bottom: '20%', left: '5%'}}>MOTOR_A: ACTIVE</div>
              <img 
                src={useBaseUrl('img/hero-robot.png')} 
                alt="Humanoid Robot" 
                className={styles.heroImage}
                style={{filter: 'drop-shadow(0 0 20px rgba(138, 235, 255, 0.4))'}}
              />
            </div>
          </div>
        </div>
      </div>
    </header>
  );
}

export default function Home() {
  return (
    <Layout title="Home" description="Physical AI & Humanoid Robots Book">
      <HomepageHeader />
      
      <main className={styles.mainContainer}>
        {/* LEARNING PATH SECTION */}
        <div style={{background: 'transparent'}}>
          <div className="container text--center">
              <h2 style={{fontSize: '2.5rem', marginBottom: '1rem', color: 'var(--ifm-color-primary)'}}>LEARNING_PATH</h2>
              <LearningPath />
          </div>
        </div>

        {/* FEATURES SECTION */}
        <div className="container" style={{paddingBottom: '4rem'}}>
          <div className="row">
            {FEATURES.map((props, idx) => (
              <TechCard key={idx} {...props} index={idx} />
            ))}
          </div>
        </div>
      </main>

      {/* PREMIUM FOOTER */}
      <footer className={styles.premiumFooter}>
        <div className="container">
          <div className="row">
            <div className="col col--6">
              <h3 className={styles.footerBrand}>ROBOT_BOOK<span className={styles.version}>v2.6</span></h3>
              <p className={styles.footerTagline}>Building the future of Physical AI, one joint at a time.</p>
            </div>
            <div className="col col--6 text--right">
              <div className={styles.footerLinks}>
                <Link to="/docs/01-ros2-nervous-system/chapter_1_introduction">DOCUMENTATION</Link>
                <Link to="#">COMMUNITY</Link>
                <Link to="#">SYSTEM_STATUS</Link>
              </div>
              <p className={styles.copyright}>© 2026 SYNTHETIC_COMMAND_CENTER. ALL_RIGHTS_RESERVED.</p>
            </div>
          </div>
          <div className={styles.footerBottomLine} />
        </div>
      </footer>
    </Layout>
  );
}
