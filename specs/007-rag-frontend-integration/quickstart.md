# Quickstart Guide: RAG Agent Frontend Integration

**Feature**: 007-rag-frontend-integration
**Date**: 2025-12-19
**Status**: Complete

## Overview

This guide provides step-by-step instructions to quickly set up and run the RAG Agent Frontend Integration feature. Follow these steps to get the chatbot interface working with your Docusaurus site.

## Prerequisites

Before starting, ensure you have:

- Node.js 18+ installed
- npm or yarn package manager
- Python 3.8+ for the backend RAG agent
- The backend RAG agent service running locally on port 8000
- Environment variables configured for the RAG agent (QDRANT_URL, QDRANT_API_KEY, COHERE_API_KEY, OPENAI_API_KEY)

## Setup Steps

### 1. Start the RAG Agent Backend

First, ensure your RAG agent backend is running:

```bash
# Navigate to the backend rag_agent directory
cd backend/rag_agent

# Start the FastAPI server
uvicorn main:app --host 0.0.0.0 --port 8000
```

### 2. Install Frontend Dependencies

In your project root directory, install the required frontend dependencies:

```bash
# Install axios for API communication
npm install axios

# Optional: Install react-icons for UI elements
npm install react-icons
```

### 3. Create the Chatbot Components

Create the following directory structure in your Docusaurus project:

```bash
mkdir -p src/components/RagChatbot
```

### 4. Add the Chatbot Component

Create the main chatbot component at `src/components/RagChatbot/RagChatbot.jsx`:

```jsx
import React, { useState, useRef, useEffect } from 'react';
import axios from 'axios';
import './ChatStyles.css';

const RagChatbot = () => {
  const [messages, setMessages] = useState([]);
  const [inputValue, setInputValue] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [error, setError] = useState(null);
  const messagesEndRef = useRef(null);

  // API configuration
  const RAG_AGENT_URL = 'http://localhost:8000';

  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  };

  useEffect(() => {
    scrollToBottom();
  }, [messages]);

  const handleSubmit = async (e) => {
    e.preventDefault();
    if (!inputValue.trim() || isLoading) return;

    const userMessage = {
      id: Date.now(),
      type: 'user',
      content: inputValue.trim(),
      timestamp: new Date()
    };

    // Add user message to chat
    setMessages(prev => [...prev, userMessage]);
    setInputValue('');
    setIsLoading(true);
    setError(null);

    try {
      // Call RAG agent API
      const response = await axios.post(`${RAG_AGENT_URL}/query`, {
        query: userMessage.content,
        top_k: 5,
        include_metadata: true
      });

      const ragResponse = {
        id: Date.now() + 1,
        type: 'system',
        content: response.data.answer,
        sources: response.data.sources,
        contexts: response.data.retrieved_contexts,
        timestamp: new Date()
      };

      setMessages(prev => [...prev, ragResponse]);
    } catch (err) {
      setError('Failed to get response from RAG agent. Please try again.');
      console.error('API Error:', err);

      // Add error message to chat
      const errorMessage = {
        id: Date.now() + 1,
        type: 'system',
        content: 'Sorry, I encountered an error processing your request. Please try again.',
        timestamp: new Date()
      };
      setMessages(prev => [...prev, errorMessage]);
    } finally {
      setIsLoading(false);
    }
  };

  return (
    <div className="rag-chatbot-container">
      <div className="chat-header">
        <h3>AI Assistant</h3>
      </div>

      <div className="chat-messages">
        {messages.length === 0 ? (
          <div className="welcome-message">
            <p>Ask me anything about the book content! I can help explain concepts and find relevant information.</p>
          </div>
        ) : (
          messages.map((message) => (
            <div
              key={message.id}
              className={`message ${message.type === 'user' ? 'user-message' : 'system-message'}`}
            >
              <div className="message-content">
                <p>{message.content}</p>
                {message.sources && message.sources.length > 0 && (
                  <div className="sources">
                    <small>Sources: {message.sources.join(', ')}</small>
                  </div>
                )}
              </div>
            </div>
          ))
        )}

        {isLoading && (
          <div className="message system-message">
            <div className="message-content">
              <div className="typing-indicator">
                <span></span>
                <span></span>
                <span></span>
              </div>
            </div>
          </div>
        )}

        {error && (
          <div className="message system-message error-message">
            <div className="message-content">
              <p>{error}</p>
            </div>
          </div>
        )}

        <div ref={messagesEndRef} />
      </div>

      <form onSubmit={handleSubmit} className="chat-input-form">
        <input
          type="text"
          value={inputValue}
          onChange={(e) => setInputValue(e.target.value)}
          placeholder="Ask a question about the book..."
          disabled={isLoading}
          className="chat-input"
        />
        <button
          type="submit"
          disabled={isLoading || !inputValue.trim()}
          className="chat-submit-btn"
        >
          {isLoading ? 'Sending...' : 'Send'}
        </button>
      </form>
    </div>
  );
};

export default RagChatbot;
```

### 5. Add Chatbot Styles

Create the CSS file at `src/components/RagChatbot/ChatStyles.css`:

```css
.rag-chatbot-container {
  display: flex;
  flex-direction: column;
  height: 500px;
  border: 1px solid #e0e0e0;
  border-radius: 8px;
  overflow: hidden;
  font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', Roboto, sans-serif;
}

.chat-header {
  background-color: #f8f9fa;
  padding: 12px 16px;
  border-bottom: 1px solid #e0e0e0;
}

.chat-header h3 {
  margin: 0;
  color: #2563eb;
  font-size: 16px;
  font-weight: 600;
}

.chat-messages {
  flex: 1;
  overflow-y: auto;
  padding: 16px;
  background-color: #fafafa;
}

.welcome-message {
  text-align: center;
  color: #6b7280;
  font-style: italic;
  margin: 20px 0;
}

.message {
  margin-bottom: 12px;
  display: flex;
}

.user-message {
  justify-content: flex-end;
}

.system-message {
  justify-content: flex-start;
}

.message-content {
  max-width: 80%;
  padding: 10px 14px;
  border-radius: 18px;
  position: relative;
}

.user-message .message-content {
  background-color: #2563eb;
  color: white;
  border-bottom-right-radius: 4px;
}

.system-message .message-content {
  background-color: white;
  color: #374151;
  border: 1px solid #e5e7eb;
  border-bottom-left-radius: 4px;
}

.system-message.error-message .message-content {
  background-color: #fee2e2;
  border-color: #fecaca;
  color: #dc2626;
}

.sources {
  margin-top: 6px;
  font-size: 12px;
  color: #6b7280;
}

.typing-indicator {
  display: flex;
  align-items: center;
  padding: 5px 0;
}

.typing-indicator span {
  height: 8px;
  width: 8px;
  background-color: #9ca3af;
  border-radius: 50%;
  display: inline-block;
  margin: 0 2px;
  animation: typing 1.4s infinite ease-in-out;
}

.typing-indicator span:nth-child(2) {
  animation-delay: 0.2s;
}

.typing-indicator span:nth-child(3) {
  animation-delay: 0.4s;
}

@keyframes typing {
  0%, 60%, 100% { transform: translateY(0); }
  30% { transform: translateY(-5px); }
}

.chat-input-form {
  display: flex;
  padding: 12px;
  background-color: white;
  border-top: 1px solid #e0e0e0;
}

.chat-input {
  flex: 1;
  padding: 10px 12px;
  border: 1px solid #d1d5db;
  border-radius: 4px;
  font-size: 14px;
  margin-right: 8px;
}

.chat-input:focus {
  outline: none;
  border-color: #2563eb;
  box-shadow: 0 0 0 2px rgba(37, 99, 235, 0.1);
}

.chat-submit-btn {
  padding: 10px 16px;
  background-color: #2563eb;
  color: white;
  border: none;
  border-radius: 4px;
  cursor: pointer;
  font-size: 14px;
  font-weight: 500;
}

.chat-submit-btn:disabled {
  background-color: #9ca3af;
  cursor: not-allowed;
}

.chat-submit-btn:not(:disabled):hover {
  background-color: #1d4ed8;
}
```

### 6. Integrate with Docusaurus

Add the chatbot to your Docusaurus layout by importing it in your desired page or layout. For example, in a page:

```jsx
import React from 'react';
import Layout from '@theme/Layout';
import RagChatbot from '@site/src/components/RagChatbot/RagChatbot';

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
```

### 7. Add to Navigation (Optional)

To add a chatbot link to your navigation, update `docusaurus.config.js`:

```javascript
module.exports = {
  // ... other config
  themeConfig: {
    // ... other theme config
    navbar: {
      items: [
        // ... other items
        {
          to: '/chat',
          label: 'AI Assistant',
          position: 'right',
        },
      ],
    },
  },
};
```

## Running the Application

1. **Start the backend RAG agent** (if not already running):
   ```bash
   cd backend/rag_agent
   uvicorn main:app --host 0.0.0.0 --port 8000
   ```

2. **Start the Docusaurus frontend**:
   ```bash
   npm start
   ```

3. **Access the chatbot** at your Docusaurus site (typically `http://localhost:3000`)

## Testing the Integration

1. Open your Docusaurus site in a browser
2. Navigate to the page with the chatbot component
3. Type a question in the input field
4. Verify that:
   - The question appears in the chat
   - Loading indicator shows during processing
   - Response appears from the RAG agent
   - Sources are displayed when available
   - Error handling works when the backend is unavailable

## Troubleshooting

### Common Issues

**Backend not responding**: Ensure the RAG agent backend is running on port 8000 and all required environment variables are set.

**CORS errors**: If running frontend and backend on different ports, ensure the backend allows requests from the frontend origin.

**API key errors**: Verify that all required API keys (QDRANT, COHERE, OPENAI) are properly configured in the backend environment.

## Next Steps

- Customize the chatbot UI to match your site's design
- Add conversation history persistence
- Implement advanced features like message editing or follow-up questions
- Add analytics to track user interactions