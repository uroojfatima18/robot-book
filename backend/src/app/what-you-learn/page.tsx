'use client';

import React from 'react';

interface ChapterBoxProps {
  title: string;
  description: string;
  index: number;
}

const ChapterBox: React.FC<ChapterBoxProps> = ({ title, description, index }) => {
  return (
    <div
      className={`
        bg-white rounded-xl shadow-lg p-6 transition-all duration-300
        hover:shadow-xl hover:-translate-y-1 border border-gray-100
        flex flex-col h-full
      `}
    >
      <div className="flex items-center mb-4">
        <div className={`
          w-10 h-10 rounded-full flex items-center justify-center
          text-white font-bold text-lg mr-3
          ${index === 1 ? 'bg-blue-500' :
            index === 2 ? 'bg-purple-500' :
            index === 3 ? 'bg-green-500' :
            index === 4 ? 'bg-yellow-500' :
            'bg-pink-500'}
        `}>
          {index}
        </div>
        <h3 className="text-xl font-bold text-gray-800">{title}</h3>
      </div>
      <p className="text-gray-600 mt-2 flex-grow">{description}</p>
    </div>
  );
};

const WhatYouLearnPage: React.FC = () => {
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
    <div className="min-h-screen bg-gradient-to-br from-gray-50 to-gray-100 py-12 px-4 sm:px-6">
      <div className="max-w-7xl mx-auto">
        <header className="text-center mb-16">
          <h1 className="text-4xl md:text-5xl font-bold text-gray-900 mb-4">
            What You'll Learn
          </h1>
          <p className="text-xl text-gray-600 max-w-3xl mx-auto">
            Explore the fascinating world of robotics through comprehensive chapters designed to build your knowledge from basics to advanced concepts.
          </p>
        </header>

        <div className="grid grid-cols-1 md:grid-cols-2 lg:grid-cols-3 gap-8">
          {chapters.map((chapter, index) => (
            <ChapterBox
              key={index}
              title={chapter.title}
              description={chapter.description}
              index={index + 1}
            />
          ))}
        </div>
      </div>
    </div>
  );
};

export default WhatYouLearnPage;