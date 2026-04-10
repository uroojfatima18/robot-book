import React from 'react';
import BrowserOnly from '@docusaurus/BrowserOnly';

export default function Root({children}) {
  return (
    <>
      {children}
      <BrowserOnly>
        {() => {
          // Dynamic requires to prevent SSR crashes on documentation pages
          const { AuthProvider } = require('../components/AuthContext');
          const Chatbot = require('../components/Chatbot').default;
          return (
            <AuthProvider>
              <Chatbot />
            </AuthProvider>
          );
        }}
      </BrowserOnly>
    </>
  );
}
