import React from 'react';
import type { Metadata } from 'next';

export const metadata: Metadata = {
  title: 'What You\'ll Learn | Robotics Course',
  description: 'Explore the chapters you will learn in our comprehensive robotics course',
};

const WhatYouLearnLayout: React.FC<{ children: React.ReactNode }> = ({ children }) => {
  return (
    <div className="min-h-screen bg-gradient-to-br from-gray-50 to-gray-100">
      {children}
    </div>
  );
};

export default WhatYouLearnLayout;