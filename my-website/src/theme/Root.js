import React from 'react';
import Chatbot from '@site/src/components/Chatbot';
import { AuthProvider } from '@site/src/components/AuthContext';

// Integration of Auth and Global Chatbot
export default function Root({children}) {
  return (
    <AuthProvider>
      {children}
      <Chatbot />
    </AuthProvider>
  );
}
