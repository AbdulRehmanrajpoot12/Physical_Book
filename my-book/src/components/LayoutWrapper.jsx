import React from 'react';
import FloatingChatbot from '../components/RagChatbot/FloatingChatbot';

// This component wraps the entire app to add the floating chatbot to all pages
const LayoutWrapper = ({ children }) => {
  return (
    <>
      {children}
      <FloatingChatbot />
    </>
  );
};

export default LayoutWrapper;