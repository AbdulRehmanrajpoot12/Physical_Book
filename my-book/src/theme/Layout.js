import React from 'react';
import OriginalLayout from '@theme-original/Layout';
import FloatingChatbot from '../components/RagChatbot/FloatingChatbot';

// This component wraps the original Docusaurus layout to add the floating chatbot to all pages
export default function LayoutWrapper(props) {
  return (
    <>
      <OriginalLayout {...props} />
      <FloatingChatbot />
    </>
  );
}