import React from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import styles from './styles.module.css';

const ChapterBox = ({ title, description, index }) => {
  const colors = [
    'bg-blue-500',
    'bg-purple-500',
    'bg-green-500',
    'bg-yellow-500',
    'bg-pink-500'
  ];

  return (
    <div className={clsx('card', styles.chapterBox)}>
      <div className="card__header">
        <div className={clsx('avatar', styles.avatar)}>
          <div className={clsx('avatar__intro', styles.avatarIntro)}>
            <h3 className="avatar__name">{index}. {title}</h3>
          </div>
        </div>
      </div>
      <div className="card__body">
        <p>{description}</p>
      </div>
    </div>
  );
};

function WhatYouLearnPage() {
  const { siteConfig } = useDocusaurusContext();

  const chapters = [
    {
      title: "Introduction to Robotics",
      description: "Learn the fundamentals of robotics including basic concepts, history, and applications in various industries."
    },
    {
      title: "ROS 2 Fundamentals",
      description: "Understand the Robot Operating System (ROS 2) architecture, nodes, topics, services, and actions."
    },
    {
      title: "Navigation and Path Planning",
      description: "Explore navigation algorithms, SLAM, and path planning techniques for autonomous robots."
    },
    {
      title: "Computer Vision for Robots",
      description: "Discover how robots perceive their environment using cameras, image processing, and computer vision techniques."
    },
    {
      title: "AI and Machine Learning in Robotics",
      description: "Learn how artificial intelligence and machine learning enhance robotic capabilities and decision-making."
    }
  ];

  return (
    <Layout
      title={`What You'll Learn`}
      description="Explore the chapters you will learn in our comprehensive robotics course">
      <main>
        <div className={clsx('container', 'margin-vert--xl', styles.whatYouLearn)}>
          <div className="row">
            <div className="col col--12">
              <header className={clsx('text--center', styles.header)}>
                <h1 className="hero__title">What You'll Learn</h1>
                <p className="hero__subtitle">
                  Explore the fascinating world of robotics through comprehensive chapters designed to build your knowledge from basics to advanced concepts.
                </p>
              </header>
            </div>
          </div>

          <div className="row" style={{ marginTop: '2rem' }}>
            {chapters.map((chapter, index) => (
              <div key={index} className="col col--4 margin-bottom--lg">
                <ChapterBox
                  title={chapter.title}
                  description={chapter.description}
                  index={index + 1}
                />
              </div>
            ))}
          </div>
        </div>
      </main>
    </Layout>
  );
}

export default WhatYouLearnPage;