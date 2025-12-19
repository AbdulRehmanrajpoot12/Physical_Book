import React from 'react';
import Layout from '@theme/Layout';
import RagChatbot from '../components/RagChatbot/RagChatbot';

export default function ChatPage() {
  return (
    <Layout title="AI Assistant" description="Chat with the RAG agent about book content">
      <div className="container margin-vert--lg">
        <div className="row">
          <div className="col col--8 col--offset-2">
            <h1>AI Assistant</h1>
            <p>Ask questions about the book content and get contextual answers.</p>
            <RagChatbot />
          </div>
        </div>
      </div>
    </Layout>
  );
}